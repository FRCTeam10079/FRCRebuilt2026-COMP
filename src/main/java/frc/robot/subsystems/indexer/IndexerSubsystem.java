// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  // ==================== STATE MACHINE ====================

  public enum WantedState {
    OFF,
    FEED,
    REVERSE,
    INDEX
  }

  private enum SystemState {
    IDLE,
    FEEDING,
    REVERSING,
    INDEXING
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.IDLE;

  public IndexerSubsystem(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    systemState = handleStateTransitions();
    applyStates();

    Logger.recordOutput("Indexer/WantedState", wantedState);
    Logger.recordOutput("Indexer/SystemState", systemState);
  }

  // ==================== STATE TRANSITIONS ====================

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case FEED -> SystemState.FEEDING;
      case REVERSE -> SystemState.REVERSING;
      case INDEX -> SystemState.INDEXING;
      case OFF -> SystemState.IDLE;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case FEEDING:
        io.setFeederVelocity(IndexerConstants.kFeederTargetRPM / 60.0);
        io.setSpindexerVelocity(IndexerConstants.kSpindexerTargetRPM / 60.0);
        break;
      case REVERSING:
        io.setFeederVelocity(IndexerConstants.kFeederReverseRPM / 60.0);
        io.setSpindexerVelocity(IndexerConstants.kSpindexerReverseRPM / 60.0);
        break;
      case INDEXING:
        // Spindexer only — rotate pieces into feeder path without shooting
        io.setFeederVelocity(0);
        io.setSpindexerVelocity(IndexerConstants.kSpindexerTargetRPM / 60.0);
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

  public Command feedCommand() {
    return startEnd(() -> setWantedState(WantedState.FEED), () -> setWantedState(WantedState.OFF))
        .withName("Indexer Feed");
  }

  public Command reverseCommand() {
    return startEnd(
            () -> setWantedState(WantedState.REVERSE), () -> setWantedState(WantedState.OFF))
        .withName("Indexer Reverse");
  }

  public Command stopCommand() {
    return runOnce(() -> setWantedState(WantedState.OFF)).withName("Indexer Stop");
  }

  // ==================== TELEMETRY ====================

  public double getFeederSupplyCurrentAmps() {
    return inputs.feederSupplyCurrentAmps;
  }

  public double getSpindexerSupplyCurrentAmps() {
    return inputs.spindexerSupplyCurrentAmps;
  }

  public double getFeederStatorCurrentAmps() {
    return inputs.feederStatorCurrentAmps;
  }

  public double getSpindexerStatorCurrentAmps() {
    return inputs.spindexerStatorCurrentAmps;
  }

  public double getFeederVoltageVolts() {
    return inputs.feederVoltageVolts;
  }

  public double getSpindexerVoltageVolts() {
    return inputs.spindexerVoltageVolts;
  }
}
