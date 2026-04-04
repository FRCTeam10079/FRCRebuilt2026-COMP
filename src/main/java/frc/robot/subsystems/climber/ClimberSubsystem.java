// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert climberMotorDisconnectedAlert =
      new Alert("Climber motor disconnected, climb may fail", AlertType.kError);

  // ==================== STATE MACHINE ====================

  public enum WantedState {
    IDLE,
    EXTEND,
    RETRACT,
    ABORT
  }

  public enum SystemState {
    /** Motor off, brake holds position. */
    IDLE,
    /** Applying extend voltage, waiting for stall. */
    EXTENDING,
    /** Motor stalled at full extension, brake holds. */
    EXTENDED,
    /** Applying retract voltage, waiting for stall. */
    RETRACTING,
    /** Motor stalled at retract position, brake holds. */
    RETRACTED
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  // ==================== STALL DETECTION ====================
  private final Timer moveTimer = new Timer();
  private int stallCycleCount = 0;
  private static final int STALL_CYCLES_REQUIRED =
      (int) (ClimberConstants.STALL_DEBOUNCE_SECONDS / 0.02);
  private static final int RAMP_UP_CYCLES = (int) (ClimberConstants.RAMP_UP_SECONDS / 0.02);
  private int moveCycleCount = 0;

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

    // Stall detection diagnostics
    Logger.recordOutput("Climber/StallCycleCount", stallCycleCount);
    Logger.recordOutput("Climber/MoveCycleCount", moveCycleCount);
    Logger.recordOutput("Climber/StallConditionMet", isStallCondition());
  }

  // ==================== STATE TRANSITIONS ====================

  private SystemState handleStateTransitions(SystemState previous) {
    switch (wantedState) {
      case IDLE:
      case ABORT:
        resetStallDetection();
        return SystemState.IDLE;

      case EXTEND:
        if (previous == SystemState.EXTENDED) {
          return SystemState.EXTENDED;
        }
        if (previous != SystemState.EXTENDING) {
          resetStallDetection();
        }
        moveCycleCount++;
        if (checkStalled()) {
          return SystemState.EXTENDED;
        }
        return SystemState.EXTENDING;

      case RETRACT:
        if (previous == SystemState.RETRACTED) {
          return SystemState.RETRACTED;
        }
        if (previous != SystemState.RETRACTING) {
          resetStallDetection();
        }
        moveCycleCount++;
        if (checkStalled()) {
          return SystemState.RETRACTED;
        }
        return SystemState.RETRACTING;

      default:
        return SystemState.IDLE;
    }
  }

  private void applyStates() {
    switch (systemState) {
      case EXTENDING:
        io.setVoltage(ClimberConstants.EXTEND_VOLTAGE);
        break;
      case RETRACTING:
        io.setVoltage(ClimberConstants.RETRACT_VOLTAGE);
        break;
      case EXTENDED:
      case RETRACTED:
      case IDLE:
      default:
        io.stop();
        break;
    }
  }

  // ==================== STALL DETECTION ====================

  private void resetStallDetection() {
    moveCycleCount = 0;
    stallCycleCount = 0;
  }

  private boolean isStallCondition() {
    return inputs.statorCurrentAmps >= ClimberConstants.STALL_CURRENT_THRESHOLD_AMPS
        && Math.abs(inputs.velocityRPS) < ClimberConstants.STALL_VELOCITY_THRESHOLD_RPS;
  }

  private boolean checkStalled() {
    if (moveCycleCount < RAMP_UP_CYCLES) {
      stallCycleCount = 0;
      return false;
    }

    if (isStallCondition()) {
      stallCycleCount++;
    } else {
      stallCycleCount = 0;
    }

    return stallCycleCount >= STALL_CYCLES_REQUIRED;
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

  public boolean isExtended() {
    return systemState == SystemState.EXTENDED;
  }

  public boolean isRetracted() {
    return systemState == SystemState.RETRACTED;
  }

  public double getPositionRotations() {
    return inputs.positionRotations;
  }

  // ==================== COMMAND FACTORIES ====================

  public Command extendCommand() {
    return run(() -> setWantedState(WantedState.EXTEND))
        .until(this::isExtended)
        .finallyDo(interrupted -> {
          if (interrupted) {
            setWantedState(WantedState.IDLE);
          }
        })
        .withName("Climber Extend");
  }

  public Command retractCommand() {
    return run(() -> setWantedState(WantedState.RETRACT))
        .until(this::isRetracted)
        .finallyDo(interrupted -> {
          if (interrupted) {
            setWantedState(WantedState.IDLE);
          }
        })
        .withName("Climber Retract");
  }

  public Command abortCommand() {
    return Commands.runOnce(() -> setWantedState(WantedState.IDLE), this).withName("Climber Abort");
  }

  public Command stopCommand() {
    return abortCommand();
  }
}
