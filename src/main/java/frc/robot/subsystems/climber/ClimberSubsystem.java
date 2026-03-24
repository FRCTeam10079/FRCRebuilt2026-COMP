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
    IDLE,
    EXTEND,
    RETRACT,
    CLIMB
  }

  private enum SystemState {
    IDLE,
    EXTENDING,
    RETRACTING,
    CLIMBING
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    systemState = handleStateTransitions();
    applyStates();

    Logger.recordOutput("Climber/WantedState", wantedState);
    Logger.recordOutput("Climber/SystemState", systemState);
  }

  // ==================== STATE TRANSITIONS ====================

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case EXTEND -> SystemState.EXTENDING;
      case RETRACT -> SystemState.RETRACTING;
      case CLIMB -> SystemState.CLIMBING;
      case IDLE -> SystemState.IDLE;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case EXTENDING:
        io.setVoltage(ClimberConstants.CLIMBER_EXTEND_SPEED * 12.0);
        break;
      case RETRACTING:
      case CLIMBING:
        io.setVoltage(ClimberConstants.CLIMBER_RETRACT_SPEED * 12.0);
        break;
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

  // ==================== COMMAND FACTORIES ====================

  public Command extendCommand() {
    return startEnd(
            () -> setWantedState(WantedState.EXTEND), () -> setWantedState(WantedState.IDLE))
        .withName("Climber Extend");
  }

  public Command retractCommand() {
    return startEnd(
            () -> setWantedState(WantedState.RETRACT), () -> setWantedState(WantedState.IDLE))
        .withName("Climber Retract");
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> setWantedState(WantedState.IDLE)).withName("Climber Stop");
  }
}
