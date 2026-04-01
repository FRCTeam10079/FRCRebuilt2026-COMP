// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  // ==================== STATE MACHINE ====================

  public enum WantedState {
    /** Do nothing - motor stopped, brake mode holds position. */
    IDLE,
    /** Pay rope out until hook is fully extended. */
    EXTEND,
    /** Wind rope in to lift the robot until scored position. */
    CLIMB,
    /** Stop immediately and return to idle from any state. */
    ABORT
  }

  public enum SystemState {
    /** Motor off, brake holds position. */
    IDLE,
    /** Running extend voltage, watching for full extension position. */
    EXTENDING,
    /** Mechanism fully extended, motor stopped, waiting for climb command. */
    EXTENDED,
    /** Running retract voltage, watching for scored position. */
    CLIMBING,
    /** Climb complete - motor off, brake holds robot weight. */
    HELD
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private double requestedVolts = 0.0;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    systemState = handleStateTransitions();
    applyStates();

    Logger.recordOutput("Climber/WantedState", wantedState.name());
    Logger.recordOutput("Climber/SystemState", systemState.name());
    Logger.recordOutput("Climber/RequestedVolts", requestedVolts);
    Logger.recordOutput("Climber/PositionRotations", inputs.positionRotations);
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

      case CLIMB:
        // Can only climb if we have extended (or are already climbing/held)
        if (systemState == SystemState.EXTENDED
            || systemState == SystemState.CLIMBING
            || systemState == SystemState.HELD) {
          if (inputs.positionRotations
              <= ClimberConstants.CLIMB_SCORED_ROTATIONS
                  + ClimberConstants.POSITION_TOLERANCE_ROTATIONS) {
            return SystemState.HELD;
          }
          return SystemState.CLIMBING;
        }
        // Not extended yet — stay in current state
        return systemState;

      case ABORT:
        return SystemState.IDLE;

      default:
        return SystemState.IDLE;
    }
  }

  private void applyStates() {
    switch (systemState) {
      case EXTENDING:
        requestedVolts = ClimberConstants.EXTEND_VOLTAGE;
        io.setVoltage(requestedVolts);
        break;
      case CLIMBING:
        requestedVolts = ClimberConstants.RETRACT_VOLTAGE;
        io.setVoltage(requestedVolts);
        break;
      case EXTENDED:
      case HELD:
      case IDLE:
      default:
        requestedVolts = 0.0;
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

  /** True when the mechanism has reached full extension and is waiting for climb. */
  public boolean isExtended() {
    return systemState == SystemState.EXTENDED;
  }

  /** True when the climb is complete and the robot is being held. */
  public boolean isClimbComplete() {
    return systemState == SystemState.HELD;
  }

  /** Current motor position in rotor rotations (for logging / dashboard). */
  public double getPositionRotations() {
    return inputs.positionRotations;
  }

  // ==================== COMMAND FACTORIES ====================

  /**
   * Extend the climber until the hook is fully deployed. Returns to IDLE if interrupted. The state
   * machine auto-transitions to EXTENDED when the position threshold is reached, at which point the
   * command ends.
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
   * Retract the climber to lift the robot. Runs until the scored position is reached, then holds.
   * If interrupted, aborts back to IDLE. On successful completion, the state machine stays in HELD
   * (motor off, brake holds).
   */
  public Command climbCommand() {
    return run(() -> setWantedState(WantedState.CLIMB))
        .until(this::isClimbComplete)
        .finallyDo(interrupted -> {
          if (interrupted) {
            setWantedState(WantedState.IDLE);
          }
          // On normal completion: state stays CLIMB -> HELD, which keeps brake engaged
        })
        .withName("Climber Climb");
  }

  /**
   * Full autonomous climb: extend immediately, then retract without pausing. Prioritizes completing
   * the climb quickly. Returns to IDLE if interrupted at any point.
   */
  public Command autoClimbCommand() {
    return Commands.sequence(extendCommand(), climbCommand()).withName("Climber Auto Climb");
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
