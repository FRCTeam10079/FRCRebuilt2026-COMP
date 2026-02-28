// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.networked.NetworkedTalonFX;

/**
 * Intake pivot arm subsystem. Controls the angular position of the intake
 * mechanism between a
 * stowed position and a deployed (intake) position using closed-loop position
 * control.
 */
public class PivotSubsystem extends SubsystemBase {

  private final NetworkedTalonFX m_pivotMotor = new NetworkedTalonFX(IntakeConstants.Pivot.MOTOR_ID, Constants.kCANBus);
  private Angle m_pivotSetpoint;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  // Stall detection state (only active while stowing)
  private boolean m_isStowing = false;
  private boolean m_isStalled = false;
  private final Timer m_stallTimer = new Timer();
  StatusSignal<Current> statorCurrentSignal = m_pivotMotor.getStatorCurrent();
  StatusSignal<Angle> rotorPositionSignal = m_pivotMotor.getRotorPosition();

  // Idle detection - switch to NeutralOut (brake mode holds mechanically)
  // when the pivot has been at setpoint long enough. Saves power.
  private boolean m_isIdle = false;
  private double m_atSetpointSinceTime = 0.0;

  public PivotSubsystem() {
    configureMotors();
    // Prevent pivot from moving on startup - set setpoint to current position
    m_pivotSetpoint = getPivotPosition();
  }

  /**
   * Configure the pivot motor with PID gains, current limits, and soft limits.
   */
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

    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(IntakeConstants.Pivot.INTAKE_POSITION);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(IntakeConstants.Pivot.STOWED_POSITION);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    m_pivotMotor.applyConfiguration(config);
  }

  /**
   * Set the pivot arm target position.
   *
   * @param position target position in rotor rotations
   */
  private void setPivotSetpoint(Angle position) {
    m_pivotSetpoint = position;
    m_positionVoltage.withPosition(m_pivotSetpoint);
  }

  /** Deploy the pivot arm to the intake (pickup) position. */
  public void deployPivot() {
    m_isStowing = false;
    m_isStalled = false;
    wakeFromIdle();
    setPivotSetpoint(IntakeConstants.Pivot.INTAKE_POSITION);
  }

  /** Stow the pivot arm to the retracted position. */
  public void stowPivot() {
    m_isStowing = true;
    m_isStalled = false;
    m_stallTimer.stop();
    setPivotSetpoint(IntakeConstants.Pivot.STOWED_POSITION);
  }

  /** Wake the motor from idle state so it resumes closed-loop control. */
  private void wakeFromIdle() {
    m_isIdle = false;
    m_atSetpointSinceTime = 0.0;
  }

  /**
   * @return true if the pivot detected a stall during stowing and is holding
   *         position.
   */
  public boolean isStalled() {
    return m_isStalled;
  }

  public boolean reachedSetpoint() {
    return getPivotPosition().isNear(m_pivotSetpoint, IntakeConstants.Pivot.DEPLOY_TOLERANCE);
  }

  public Angle getPivotPosition() {
    return rotorPositionSignal.getValue();
  }

  @Override
  public void periodic() {
    m_pivotMotor.periodic();

    detectStall();

    // --- Idle detection: switch to NeutralOut once at setpoint for long enough ---
    // Brake mode holds position mechanically, saving power. Inspired by 1678's
    // approach of using brake mode + zero-output when at target, rather than
    // continuous PID commanding (which wastes current, especially with kG=0).
    if (!m_isIdle) {
      if (reachedSetpoint() && !m_isStowing) {
        if (m_atSetpointSinceTime == 0.0) {
          m_atSetpointSinceTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - m_atSetpointSinceTime
            >= IntakeConstants.Pivot.IDLE_DEBOUNCE_SECONDS) {
          m_isIdle = true;
        }
      } else {
        m_atSetpointSinceTime = 0.0;
      }
    }

    // Send motor command: NeutralOut when idle, PositionVoltage otherwise
    if (m_isIdle) {
      m_pivotMotor.setControl(m_neutralRequest);
    } else {
      m_pivotMotor.setControl(m_positionVoltage.withPosition(m_pivotSetpoint));
    }

    SmartDashboard.putNumber("Pivot/setpoint", m_pivotSetpoint.in(Rotations));
    SmartDashboard.putNumber("Pivot/position", getPivotPosition().in(Rotations));
    SmartDashboard.putBoolean("Pivot/reachedSetpoint?", reachedSetpoint());
    SmartDashboard.putBoolean("Pivot/isStalled", m_isStalled);
    SmartDashboard.putBoolean("Pivot/isIdle", m_isIdle);
    SmartDashboard.putNumber(
        "Pivot/statorCurrent", statorCurrentSignal.getValue().in(Amps));
  }

  private void detectStall() {
    if (!m_isStowing || m_isStalled || reachedSetpoint()) {
      return;
    }

    var statorCurrent = statorCurrentSignal.getValue();
    if (statorCurrent.lte(IntakeConstants.Pivot.STALL_CURRENT_THRESHOLD)) {
      // Current dropped below threshold — reset timer
      m_stallTimer.restart();
      return;
    }

    if (m_stallTimer.hasElapsed(IntakeConstants.Pivot.STALL_TIME_THRESHOLD)) {
      // Stall confirmed — hold current position instead of fighting the obstruction
      m_isStalled = true;
      m_pivotSetpoint = getPivotPosition();
    }
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
