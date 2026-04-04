// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
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
    /** Do nothing - motor stopped, brake mode holds position. */
    IDLE,
    /** Move to full extension position (164 rotations). */
    EXTEND,
    /** Retract to climb position (~82 rotations) to lift robot. */
    RETRACT,
    /** Stop immediately and return to idle from any state. */
    ABORT
  }

  public enum SystemState {
    /** Motor off, brake holds position. */
    IDLE,
    /** Moving toward full extension via Motion Magic. */
    EXTENDING,
    /** At full extension, actively holding position. */
    EXTENDED,
    /** Moving toward retract/climb position via Motion Magic. */
    RETRACTING,
    /** At retract/climb position, actively holding position. */
    RETRACTED
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private double activeTargetRotations = ClimberConstants.FULL_RETRACT_ROTATIONS;

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

    systemState = handleStateTransitions();
    applyStates();

    Logger.recordOutput("Climber/WantedState", wantedState.name());
    Logger.recordOutput("Climber/SystemState", systemState.name());
    Logger.recordOutput("Climber/PositionRotations", inputs.positionRotations);
    Logger.recordOutput("Climber/TargetRotations", activeTargetRotations);
    Logger.recordOutput("Climber/MotorConnected", inputs.motorConnected);
  }

  // ==================== STATE TRANSITIONS ====================

  private SystemState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return SystemState.IDLE;

      case EXTEND:
        if (inputs.positionRotations
            >= ClimberConstants.FULL_EXTEND_ROTATIONS
                - ClimberConstants.POSITION_TOLERANCE_ROTATIONS) {
          return SystemState.EXTENDED;
        }
        return SystemState.EXTENDING;

      case RETRACT:
        if (inputs.positionRotations
            <= ClimberConstants.CLIMB_RETRACT_ROTATIONS
                + ClimberConstants.POSITION_TOLERANCE_ROTATIONS) {
          return SystemState.RETRACTED;
        }
        return SystemState.RETRACTING;

      case ABORT:
        return SystemState.IDLE;

      default:
        return SystemState.IDLE;
    }
  }

  private void applyStates() {
    switch (systemState) {
      case EXTENDING:
        activeTargetRotations = ClimberConstants.FULL_EXTEND_ROTATIONS;
        io.setPosition(activeTargetRotations);
        break;
      case EXTENDED:
        activeTargetRotations = ClimberConstants.FULL_EXTEND_ROTATIONS;
        io.setPosition(activeTargetRotations);
        break;
      case RETRACTING:
        activeTargetRotations = ClimberConstants.CLIMB_RETRACT_ROTATIONS;
        io.setPosition(activeTargetRotations);
        break;
      case RETRACTED:
        activeTargetRotations = ClimberConstants.CLIMB_RETRACT_ROTATIONS;
        io.setPosition(activeTargetRotations);
        break;
      case IDLE:
      default:
        io.stop();
        activeTargetRotations = inputs.positionRotations;
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

  /** True when the mechanism has reached full extension and is holding. */
  public boolean isExtended() {
    return systemState == SystemState.EXTENDED;
  }

  /** True when the mechanism has retracted to the climb position and is holding. */
  public boolean isRetracted() {
    return systemState == SystemState.RETRACTED;
  }

  /** Current motor position in rotor rotations (for logging / dashboard). */
  public double getPositionRotations() {
    return inputs.positionRotations;
  }

  // ==================== COMMAND FACTORIES ====================

  /**
   * Extend the climber to full extension (164 rotations). The command ends when the position
   * threshold is reached. If interrupted, returns to IDLE (brake holds).
   */
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

  /**
   * Retract the climber to the climb position (~82 rotations) to lift the robot. The command ends
   * when the position threshold is reached. If interrupted, returns to IDLE (brake holds). On
   * completion the state machine stays in RETRACT -> RETRACTED, actively holding position.
   */
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

  /** Abort the climb from any state. Motor stops immediately, brake holds. */
  public Command abortCommand() {
    return Commands.runOnce(() -> setWantedState(WantedState.IDLE), this).withName("Climber Abort");
  }

  /** Stop command — alias for abort for backward compatibility. */
  public Command stopCommand() {
    return abortCommand();
  }
}
