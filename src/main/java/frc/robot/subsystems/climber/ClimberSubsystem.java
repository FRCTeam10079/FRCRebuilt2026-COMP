// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ClimberSubsystem extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert climberMotorDisconnectedAlert =
      new Alert("Climber motor disconnected, climb may fail", AlertType.kError);

  // ==================== TUNABLE DURATIONS (NetworkTables) ====================
  private static final LoggedNetworkNumber extendDuration = new LoggedNetworkNumber(
      "/Tuning/Climb/ExtendDurationSeconds", ClimberConstants.EXTEND_DURATION_SECONDS);
  private static final LoggedNetworkNumber holdDuration = new LoggedNetworkNumber(
      "/Tuning/Climb/HoldDurationSeconds", ClimberConstants.HOLD_DURATION_SECONDS);
  private static final LoggedNetworkNumber retractDuration = new LoggedNetworkNumber(
      "/Tuning/Climb/RetractDurationSeconds", ClimberConstants.RETRACT_DURATION_SECONDS);

  // ==================== STATE MACHINE ====================

  public enum WantedState {
    IDLE,
    CLIMB,
    MANUAL,
    ABORT
  }

  public enum SystemState {
    IDLE,
    EXTENDING,
    HOLDING,
    RETRACTING,
    RETRACTED,
    MANUAL
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  // ==================== TIMER ====================
  private final Timer phaseTimer = new Timer();

  // ==================== MANUAL CONTROL ====================
  private double manualVoltage = 0.0;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (RobotBase.isReal()) {
      climberMotorDisconnectedAlert.set(!inputs.motorConnected);
    }

    SystemState previousState = systemState;
    systemState = handleStateTransitions(previousState);
    applyStates();

    Logger.recordOutput("Climber/WantedState", wantedState.name());
    Logger.recordOutput("Climber/SystemState", systemState.name());
    Logger.recordOutput("Climber/PositionRotations", inputs.positionRotations);
    Logger.recordOutput("Climber/MotorConnected", inputs.motorConnected);
    Logger.recordOutput("Climber/AppliedVoltage", inputs.appliedVoltage);
    Logger.recordOutput("Climber/StatorCurrentAmps", inputs.statorCurrentAmps);
    Logger.recordOutput("Climber/VelocityRPS", inputs.velocityRPS);
    Logger.recordOutput("Climber/DutyCycle", inputs.dutyCycle);
    Logger.recordOutput("Climber/SupplyVoltage", inputs.supplyVoltage);
    Logger.recordOutput("Climber/SupplyCurrentAmps", inputs.supplyCurrentAmps);
    Logger.recordOutput("Climber/TempCelsius", inputs.tempCelsius);

    Logger.recordOutput("Climber/PhaseTimerElapsed", phaseTimer.get());
    Logger.recordOutput("Climber/ManualVoltage", manualVoltage);
  }

  // ==================== STATE TRANSITIONS ====================

  private SystemState handleStateTransitions(SystemState previous) {
    if (wantedState == WantedState.ABORT) {
      phaseTimer.stop();
      phaseTimer.reset();
      wantedState = WantedState.IDLE;
      return SystemState.IDLE;
    }

    if (wantedState == WantedState.IDLE
        && previous != SystemState.EXTENDING
        && previous != SystemState.HOLDING
        && previous != SystemState.RETRACTING) {
      return SystemState.IDLE;
    }

    if (wantedState == WantedState.MANUAL) {
      if (previous != SystemState.MANUAL) {
        phaseTimer.stop();
        phaseTimer.reset();
      }
      return SystemState.MANUAL;
    }

    if (wantedState == WantedState.CLIMB) {
      switch (previous) {
        case IDLE:
        case RETRACTED:
          phaseTimer.restart();
          return SystemState.EXTENDING;

        case EXTENDING:
          if (phaseTimer.hasElapsed(extendDuration.get())) {
            phaseTimer.restart();
            return SystemState.HOLDING;
          }
          return SystemState.EXTENDING;

        case HOLDING:
          if (phaseTimer.hasElapsed(holdDuration.get())) {
            phaseTimer.restart();
            return SystemState.RETRACTING;
          }
          return SystemState.HOLDING;

        case RETRACTING:
          if (phaseTimer.hasElapsed(retractDuration.get())) {
            phaseTimer.stop();
            phaseTimer.reset();
            return SystemState.RETRACTED;
          }
          return SystemState.RETRACTING;

        case MANUAL:
          phaseTimer.restart();
          return SystemState.EXTENDING;

        default:
          return previous;
      }
    }

    if (previous == SystemState.RETRACTED) {
      if (wantedState == WantedState.IDLE) {
        return SystemState.IDLE;
      }
      return SystemState.RETRACTED;
    }

    return previous;
  }

  private void applyStates() {
    switch (systemState) {
      case EXTENDING:
        io.setVoltage(ClimberConstants.EXTEND_VOLTAGE);
        break;
      case RETRACTING:
        io.setVoltage(ClimberConstants.RETRACT_VOLTAGE);
        break;
      case MANUAL:
        io.setVoltage(manualVoltage);
        break;
      case HOLDING:
      case RETRACTED:
      case IDLE:
      default:
        io.stop();
        break;
    }
  }

  // ==================== PUBLIC API ====================

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public SystemState getSystemState() {
    return systemState;
  }

  public void setManualVoltage(double voltage) {
    this.manualVoltage = MathUtil.clamp(
        voltage, ClimberConstants.PEAK_REVERSE_VOLTAGE, ClimberConstants.PEAK_FORWARD_VOLTAGE);
  }

  public boolean isRetracted() {
    return systemState == SystemState.RETRACTED;
  }

  public boolean isIdle() {
    return systemState == SystemState.IDLE;
  }

  public double getPositionRotations() {
    return inputs.positionRotations;
  }

  // ==================== COMMAND FACTORIES ====================

  public Command timerClimbCommand() {
    return run(() -> setWantedState(WantedState.CLIMB))
        .until(this::isRetracted)
        .finallyDo(interrupted -> {
          if (interrupted) {
            setWantedState(WantedState.IDLE);
          }
        })
        .withName("Climber Timer Climb");
  }

  public Command manualControlCommand(DoubleSupplier stickInput) {
    return run(() -> {
          setWantedState(WantedState.MANUAL);
          setManualVoltage(stickInput.getAsDouble() * ClimberConstants.EXTEND_VOLTAGE);
        })
        .finallyDo(interrupted -> {
          setManualVoltage(0.0);
          setWantedState(WantedState.IDLE);
        })
        .withName("Climber Manual Control");
  }

  public Command abortCommand() {
    return Commands.runOnce(() -> setWantedState(WantedState.ABORT), this)
        .withName("Climber Abort");
  }

  public Command stopCommand() {
    return abortCommand();
  }
}
