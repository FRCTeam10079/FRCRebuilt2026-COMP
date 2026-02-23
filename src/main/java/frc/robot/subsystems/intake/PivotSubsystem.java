// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.networked.NetworkedTalonFX;

/**
 * Intake pivot arm subsystem. Controls the angular position of the intake mechanism between a
 * stowed position and a deployed (intake) position using closed-loop position control.
 */
public class PivotSubsystem extends SubsystemBase {

  private final NetworkedTalonFX m_pivotMotor =
      new NetworkedTalonFX(IntakeConstants.Pivot.MOTOR_ID, Constants.kCANBus);
  private double m_pivotSetpoint;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(m_pivotSetpoint);

  // Stall detection state (only active while stowing)
  private boolean m_isStowing = false;
  private boolean m_isStalled = false;
  private double m_stallStartTime = 0.0;

  public PivotSubsystem() {
    configureMotors();
    // Prevent pivot from moving on startup — set setpoint to current position
    m_pivotSetpoint = getPivotPosition();
  }

  /** Configure the pivot motor with PID gains, current limits, and soft limits. */
  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // PID constants
    config.Slot0.withGravityType(GravityTypeValue.Arm_Cosine)
        .withKA(IntakeConstants.Pivot.KA)
        .withKV(IntakeConstants.Pivot.KV)
        .withKD(IntakeConstants.Pivot.KD)
        .withKG(IntakeConstants.Pivot.KG)
        .withKS(IntakeConstants.Pivot.KS)
        .withKI(IntakeConstants.Pivot.KI)
        .withKP(IntakeConstants.Pivot.KP);

    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Pivot.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.Pivot.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.Pivot.INTAKE_POSITION;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.Pivot.STOWED_POSITION;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    m_pivotMotor.applyConfiguration(config);
  }

  /**
   * Set the pivot arm target position.
   *
   * @param position target position in rotor rotations
   */
  private void setPivotPosition(double position) {
    m_pivotSetpoint = position;
    m_positionVoltage.withPosition(m_pivotSetpoint);
  }

  /** Deploy the pivot arm to the intake (pickup) position. */
  public void deployPivot() {
    m_isStowing = false;
    m_isStalled = false;
    setPivotPosition(IntakeConstants.Pivot.INTAKE_POSITION);
  }

  /** Stow the pivot arm to the retracted position. */
  public void stowPivot() {
    m_isStowing = true;
    m_isStalled = false;
    m_stallStartTime = 0.0;
    setPivotPosition(IntakeConstants.Pivot.STOWED_POSITION);
  }

  /** @return true if the pivot detected a stall during stowing and is holding position. */
  public boolean isStalled() {
    return m_isStalled;
  }

  public boolean reachedSetpoint() {
    return Math.abs(getPivotPosition() - m_pivotSetpoint) < IntakeConstants.Pivot.DEPLOY_TOLERANCE;
  }

  public double getPivotPosition() {
    return m_pivotMotor.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    m_pivotMotor.periodic();

    // --- Stall detection (only while actively stowing and not yet at setpoint) ---
    if (m_isStowing && !m_isStalled && !reachedSetpoint()) {
      double statorCurrent = m_pivotMotor.getStatorCurrent().getValueAsDouble();
      if (statorCurrent > IntakeConstants.Pivot.STALL_CURRENT_THRESHOLD) {
        if (m_stallStartTime == 0.0) {
          m_stallStartTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - m_stallStartTime
            >= IntakeConstants.Pivot.STALL_TIME_THRESHOLD) {
          // Stall confirmed — hold current position instead of fighting the obstruction
          m_isStalled = true;
          m_pivotSetpoint = getPivotPosition();
        }
      } else {
        // Current dropped below threshold — reset timer
        m_stallStartTime = 0.0;
      }
    }

    // Always run closed-loop position control (no NeutralOut)
    m_pivotMotor.setControl(m_positionVoltage.withPosition(m_pivotSetpoint));

    SmartDashboard.putNumber("Pivot/setpoint", m_pivotSetpoint);
    SmartDashboard.putNumber("Pivot/position", getPivotPosition());
    SmartDashboard.putBoolean("Pivot/reachedSetpoint?", reachedSetpoint());
    SmartDashboard.putBoolean("Pivot/isStalled", m_isStalled);
    SmartDashboard.putNumber(
        "Pivot/statorCurrent", m_pivotMotor.getStatorCurrent().getValueAsDouble());
  }

  // ==================== COMMAND FACTORIES ====================

  /**
   * Command to deploy the pivot arm and wait until it reaches the setpoint.
   *
   * @return a deploy command that requires this subsystem
   */
  public Command deployCommand() {
    return runOnce(this::deployPivot)
        .andThen(Commands.waitUntil(this::reachedSetpoint))
        .withName("Pivot Deploy");
  }

  /**
   * Command to stow the pivot arm and wait until it reaches the setpoint.
   *
   * @return a stow command that requires this subsystem
   */
  public Command stowCommand() {
    return runOnce(this::stowPivot)
        .andThen(Commands.waitUntil(this::reachedSetpoint))
        .withName("Pivot Stow");
  }
}
