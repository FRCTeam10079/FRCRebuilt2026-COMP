// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Dual-motor indexer subsystem with independent feeder and spindexer control.
 *
 * <p>The feeder is a fast wheel that shoots game pieces upward, while the spindexer is a slower
 * floor wheel that rotates pieces into the feeder path.
 */
public class IndexerSubsystem extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public IndexerSubsystem(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  /**
   * Sets the speeds of both indexer motors independently.
   *
   * @param feederRPM Target RPM for the fast feeder wheel
   * @param spindexerRPM Target RPM for the floor/spindexer wheel
   */
  public void setSpeeds(double feederRPM, double spindexerRPM) {
    io.setFeederVelocity(feederRPM / 60.0);
    io.setSpindexerVelocity(spindexerRPM / 60.0);
  }

  /** Stop both indexer motors immediately. */
  public void stop() {
    io.stop();
  }

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

  // ==================== COMMAND FACTORIES ====================

  public Command feedCommand() {
    return startEnd(
            () ->
                setSpeeds(IndexerConstants.kFeederTargetRPM, IndexerConstants.kSpindexerTargetRPM),
            this::stop)
        .withName("Indexer Feed");
  }

  public Command reverseCommand() {
    return startEnd(
            () -> setSpeeds(
                IndexerConstants.kFeederReverseRPM, IndexerConstants.kSpindexerReverseRPM),
            this::stop)
        .withName("Indexer Reverse");
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Indexer Stop");
  }

  public Command runAtSpeedsCommand(double feederRPM, double spindexerRPM) {
    return startEnd(() -> setSpeeds(feederRPM, spindexerRPM), this::stop)
        .withName("Indexer " + feederRPM + "/" + spindexerRPM + " RPM");
  }
}
