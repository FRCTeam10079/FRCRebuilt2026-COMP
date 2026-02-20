// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake wheels subsystem for collecting game pieces.
 *
 * <p>Uses a single TalonFX (Kraken X60) with velocity control for consistent intake/eject speeds
 * regardless of battery voltage.
 */
public class IntakeWheelsSubsystem extends SubsystemBase {

  private final TalonFX m_intakeMotor =
      new TalonFX(IntakeConstants.Wheels.MOTOR_ID, Constants.kCANBus);
  private final VelocityVoltage m_velocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  public IntakeWheelsSubsystem() {
    configureMotors();
  }

  /** Configure the intake motor with PID gains, current limits, and coast mode. */
  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Wheels.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.Wheels.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.withKA(IntakeConstants.Wheels.KA)
        .withKV(IntakeConstants.Wheels.KV)
        .withKD(IntakeConstants.Wheels.KD)
        .withKS(IntakeConstants.Wheels.KS)
        .withKI(IntakeConstants.Wheels.KI)
        .withKP(IntakeConstants.Wheels.KP);

    m_intakeMotor.getConfigurator().apply(config);
  }

  /** Run intake wheels inward to collect game pieces. */
  public void intakeIn() {
    m_intakeMotor.setControl(
        m_velocityRequest.withVelocity(IntakeConstants.Wheels.INTAKE_IN_RPM / 60.0));
  }

  /** Reverse intake wheels to eject stuck game pieces. */
  private void intakeOut() {
    m_intakeMotor.setControl(
        m_velocityRequest.withVelocity(IntakeConstants.Wheels.INTAKE_OUT_RPM / 60.0));
  }

  /** Stop the intake wheels (coast to stop via NeutralOut). */
  public void stop() {
    m_intakeMotor.setControl(m_neutralRequest);
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
}
