// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

/**
 * Dual-motor indexer subsystem with independent feeder and spindexer control.
 *
 * <p>The feeder is a fast wheel that shoots game pieces upward, while the spindexer is a slower
 * floor wheel that rotates pieces into the feeder path.
 */
public class IndexerSubsystem extends SubsystemBase {

  // Two independent motors
  private final TalonFX m_feederMotor;
  private final TalonFX m_spindexerMotor;

  // Two independent control requests (allows us to send different speeds)
  private final VelocityVoltage m_feederRequest = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage m_spindexerRequest = new VelocityVoltage(0).withSlot(0);

  public IndexerSubsystem() {
    m_feederMotor = new TalonFX(IndexerConstants.kFeederMotorID, new CANBus("rio"));
    m_spindexerMotor = new TalonFX(IndexerConstants.kSpindexerMotorID, new CANBus("rio"));
    configureMotors();
  }

  /** Configure both indexer motors with PID gains, current limits, and brake mode. */
  private void configureMotors() {
    // ==================== FEEDER CONFIGURATION ====================
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit;
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.Slot0 = new Slot0Configs()
        .withKP(IndexerConstants.kFeederKP)
        .withKI(IndexerConstants.kFeederKI)
        .withKD(IndexerConstants.kFeederKD)
        .withKS(IndexerConstants.kFeederKS)
        .withKV(IndexerConstants.kFeederKV)
        .withKA(IndexerConstants.kFeederKA)
        .withKG(IndexerConstants.kFeederKG);
    m_feederMotor.getConfigurator().apply(feederConfig);

    // ==================== SPINDEXER CONFIGURATION ====================
    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    spindexerConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit;
    spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    spindexerConfig.Slot0 = new Slot0Configs()
        .withKP(IndexerConstants.kSpindexerKP)
        .withKI(IndexerConstants.kSpindexerKI)
        .withKD(IndexerConstants.kSpindexerKD)
        .withKS(IndexerConstants.kSpindexerKS)
        .withKV(IndexerConstants.kSpindexerKV)
        .withKA(IndexerConstants.kSpindexerKA)
        .withKG(IndexerConstants.kSpindexerKG);
    m_spindexerMotor.getConfigurator().apply(spindexerConfig);
  }

  /**
   * Sets the speeds of both indexer motors independently.
   *
   * @param feederRPM Target RPM for the fast feeder wheel
   * @param spindexerRPM Target RPM for the floor/spindexer wheel
   */
  public void setSpeeds(double feederRPM, double spindexerRPM) {
    // 1. Convert RPM to RPS
    double feederRPS = feederRPM / 60.0;
    double spindexerRPS = spindexerRPM / 60.0;

    // 2. Send commands to motors
    m_feederMotor.setControl(m_feederRequest.withVelocity(feederRPS));
    m_spindexerMotor.setControl(m_spindexerRequest.withVelocity(spindexerRPS));
  }

  /** Stop both indexer motors immediately. */
  public void stop() {
    m_feederMotor.stopMotor();
    m_spindexerMotor.stopMotor();
  }

  // ==================== COMMAND FACTORIES ====================

  /**
   * Command to feed game pieces forward. Runs while the command is active, stops on end.
   *
   * @return a feed command that requires this subsystem
   */
  public Command feedCommand() {
    return startEnd(
            () ->
                setSpeeds(IndexerConstants.kFeederTargetRPM, IndexerConstants.kSpindexerTargetRPM),
            this::stop)
        .withName("Indexer Feed");
  }

  /**
   * Command to reverse the indexer (unjam). Runs while the command is active, stops on end.
   *
   * @return a reverse command that requires this subsystem
   */
  public Command reverseCommand() {
    return startEnd(
            () -> setSpeeds(
                IndexerConstants.kFeederReverseRPM, IndexerConstants.kSpindexerReverseRPM),
            this::stop)
        .withName("Indexer Reverse");
  }

  /**
   * Command to stop the indexer immediately.
   *
   * @return an instant stop command that requires this subsystem
   */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Indexer Stop");
  }

  /**
   * Command to run both indexer motors at custom RPMs. Stops on end.
   *
   * @param feederRPM target RPM for the feeder motor
   * @param spindexerRPM target RPM for the spindexer motor
   * @return a start-end command that requires this subsystem
   */
  public Command runAtSpeedsCommand(double feederRPM, double spindexerRPM) {
    return startEnd(() -> setSpeeds(feederRPM, spindexerRPM), this::stop)
        .withName("Indexer " + feederRPM + "/" + spindexerRPM + " RPM");
  }
}
