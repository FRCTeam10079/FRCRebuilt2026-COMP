// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake wheels subsystem for collecting game pieces.
 *
 * <p>Uses a double TalonFX (Kraken X60) with velocity control for consistent intake/eject speeds
 * regardless of battery voltage.
 */
public class IntakeWheelsSubsystem extends SubsystemBase {

  // MASTER + SLAVE
  private final TalonFX m_masterMotor =
      new TalonFX(IntakeConstants.Wheels.MOTOR_ID, Constants.kCANBus);

  private final TalonFX m_slaveMotor =
      new TalonFX(IntakeConstants.Wheels.SLAVE_MOTOR_ID, Constants.kCANBus);

  private final VelocityVoltage m_velocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(false);

  private final NeutralOut m_neutralRequest = new NeutralOut();

  private final Follower m_followerRequest;

  public IntakeWheelsSubsystem() {
    configureMotors();

    // Follower setup (slave follows master)
    m_followerRequest = new Follower(IntakeConstants.Wheels.MOTOR_ID, MotorAlignmentValue.Opposed);

    m_slaveMotor.setControl(m_followerRequest);
  }

  /** Configure the intake motor with PID gains, current limits, and coast mode. */
  private void configureMotors() {
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();

    masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    masterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    masterConfig.CurrentLimits.withSupplyCurrentLimit(IntakeConstants.Wheels.SUPPLY_CURRENT_LIMIT);
    masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    masterConfig.CurrentLimits.withStatorCurrentLimit(IntakeConstants.Wheels.STATOR_CURRENT_LIMIT);
    masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    masterConfig
        .Slot0
        .withKA(IntakeConstants.Wheels.KA)
        .withKV(IntakeConstants.Wheels.KV)
        .withKD(IntakeConstants.Wheels.KD)
        .withKS(IntakeConstants.Wheels.KS)
        .withKI(IntakeConstants.Wheels.KI)
        .withKP(IntakeConstants.Wheels.KP);

    m_masterMotor.getConfigurator().apply(masterConfig);

    // SLAVE CONFIG
    TalonFXConfiguration slaveConfig = new TalonFXConfiguration();
    slaveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_slaveMotor.getConfigurator().apply(slaveConfig);
  }

  /** Run intake wheels inward to collect game pieces. */
  public void intakeIn() {
    m_masterMotor.setControl(
        m_velocityRequest.withVelocity(IntakeConstants.Wheels.INTAKE_IN_RPM / 60.0));
  }

  /** Reverse intake wheels to eject stuck game pieces. */
  private void intakeOut() {
    m_masterMotor.setControl(
        m_velocityRequest.withVelocity(IntakeConstants.Wheels.INTAKE_OUT_RPM / 60.0));
  }

  /** Stop the intake wheels (coast to stop via NeutralOut). */
  public void stop() {
    m_masterMotor.setControl(m_neutralRequest);
  }

  public double getSupplyCurrentAmps() {
    return m_intakeMotor.getSupplyCurrent().getValueAsDouble();
  }

  public double getStatorCurrentAmps() {
    return m_intakeMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getMotorVoltageVolts() {
    return m_intakeMotor.getMotorVoltage().getValueAsDouble();
  }

  // ==================== COMMAND FACTORIES ====================

  /**
   * Command to run intake inward. Runs while active, stops on end.
   *
   * @return a start-end command that requires this subsystem
   */
  public Command intakeInCommand() {
    return startEnd(this::intakeIn, this::stop).withName("Intake In");
  }

  /**
   * Command to reverse intake (eject/unjam). Runs while active, stops on end.
   *
   * @return a start-end command that requires this subsystem
   */
  public Command intakeOutCommand() {
    return startEnd(this::intakeOut, this::stop).withName("Intake Out");
  }

  /**
   * Command to stop the intake wheels immediately.
   *
   * @return an instant stop command that requires this subsystem
   */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Intake Stop");
  }

  @Override
  public void periodic() {}
}
