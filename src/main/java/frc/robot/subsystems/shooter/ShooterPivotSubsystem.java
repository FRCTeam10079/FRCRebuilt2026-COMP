// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterPivotConstants;
import java.util.function.DoubleSupplier;

/**
 * Shooter pivot subsystem with closed-loop MotionMagic position control.
 *
 * <p>Features: - MotionMagicVoltage for smooth profiled positioning - Gravity feedforward (kG *
 * cos) to hold position against gravity - Hard-stop homing routine to calibrate the integrated
 * encoder - Software limits to protect the mechanism - Manual override fallback for operator
 * control
 */
public class ShooterPivotSubsystem extends SubsystemBase {

  private final TalonFX m_pivotMotor;

  // Control requests
  private final MotionMagicVoltage m_motionMagicRequest =
      new MotionMagicVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  // State tracking
  private boolean m_isHomed = true;
  private double m_targetAngleDegrees = ShooterPivotConstants.MIN_ANGLE_DEGREES;

  public ShooterPivotSubsystem() {
    m_pivotMotor = new TalonFX(ShooterPivotConstants.MOTOR_ID);
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Current limits
    config.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(ShooterPivotConstants.SUPPLY_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(ShooterPivotConstants.STATOR_CURRENT_LIMIT);

    // PID + FF gains (Slot 0)
    config.Slot0 = new Slot0Configs()
        .withKP(ShooterPivotConstants.KP)
        .withKI(ShooterPivotConstants.KI)
        .withKD(ShooterPivotConstants.KD)
        .withKS(ShooterPivotConstants.KS)
        .withKV(ShooterPivotConstants.KV)
        .withKG(ShooterPivotConstants.KG)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    // MotionMagic profile
    config.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(ShooterPivotConstants.MOTION_MAGIC_CRUISE_VELOCITY)
        .withMotionMagicAcceleration(ShooterPivotConstants.MOTION_MAGIC_ACCELERATION)
        .withMotionMagicJerk(ShooterPivotConstants.MOTION_MAGIC_JERK);

    // Software limits (in motor rotations)
    // Initially disabled until homing is complete
    config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ShooterPivotConstants.degreesToMotorRotations(
            ShooterPivotConstants.MAX_ANGLE_DEGREES - ShooterPivotConstants.MIN_ANGLE_DEGREES))
        .withReverseSoftLimitThreshold(0.0);

    m_pivotMotor.getConfigurator().apply(config);

    // Zero encoder on boot (will be re-zeroed by homing routine)
    m_pivotMotor.setPosition(0);
  }

  // ==================== POSITION CONTROL ====================

  /**
   * Command the pivot to a specific angle using MotionMagic.
   *
   * @param angleDegrees target angle in degrees (60-80deg range)
   */
  public void setAngle(double angleDegrees) {
    // Clamp to safe range
    angleDegrees = MathUtil.clamp(
        angleDegrees,
        ShooterPivotConstants.MIN_ANGLE_DEGREES,
        ShooterPivotConstants.MAX_ANGLE_DEGREES);
    m_targetAngleDegrees = angleDegrees;

    double motorRotations = ShooterPivotConstants.degreesToMotorRotations(
        angleDegrees - ShooterPivotConstants.MIN_ANGLE_DEGREES);
    m_pivotMotor.setControl(m_motionMagicRequest.withPosition(motorRotations));
  }

  /**
   * Get the current pivot angle in degrees.
   *
   * @return pivot angle in degrees (relative to hard stop zero)
   */
  public double getCurrentAngleDegrees() {
    return ShooterPivotConstants.motorRotationsToDegrees(
            m_pivotMotor.getPosition().getValueAsDouble())
        + ShooterPivotConstants.MIN_ANGLE_DEGREES;
  }

  /**
   * Check if the pivot is at the target angle within shooting tolerance.
   *
   * @param targetDegrees the target angle
   * @param toleranceDegrees the tolerance in degrees
   * @return true if within tolerance
   */
  public boolean isAtAngle(double targetDegrees, double toleranceDegrees) {
    return Math.abs(getCurrentAngleDegrees() - targetDegrees) <= toleranceDegrees;
  }

  /** Check if the pivot is at its current target within shooting tolerance. */
  public boolean isAtTarget() {
    return isAtAngle(m_targetAngleDegrees, ShooterPivotConstants.SHOOTING_TOLERANCE_DEGREES);
  }

  /** @return whether the pivot has been homed via hard-stop detection. */
  public boolean isHomed() {
    return m_isHomed;
  }

  /** @return the current target angle in degrees. */
  public double getTargetAngleDegrees() {
    return m_targetAngleDegrees;
  }

  // ==================== MANUAL / RAW CONTROL ====================

  /**
   * Set raw duty cycle output (for manual override or homing).
   *
   * @param output duty cycle (-1 to 1), clamped to safe range
   */
  public void setOutput(double output) {
    double clamped = MathUtil.clamp(
        output, -ShooterPivotConstants.MANUAL_MAX_OUTPUT, ShooterPivotConstants.MANUAL_MAX_OUTPUT);
    m_pivotMotor.setControl(m_dutyCycleRequest.withOutput(clamped));
  }

  public void stop() {
    m_pivotMotor.setControl(m_neutralRequest);
  }

  public double getPosition() {
    return m_pivotMotor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return m_pivotMotor.getVelocity().getValueAsDouble();
  }

  public void reZeroIfNeeded() {
    if (m_pivotMotor.getPosition().getValueAsDouble() < 0.0) {
      m_pivotMotor.setPosition(0);
    }
  }

  // ==================== HOMING ====================

  /** Enable software limits after homing is complete. Called internally after a successful home. */
  private void enableSoftwareLimits() {
    var softLimits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ShooterPivotConstants.degreesToMotorRotations(
            ShooterPivotConstants.MAX_ANGLE_DEGREES - ShooterPivotConstants.MIN_ANGLE_DEGREES))
        .withReverseSoftLimitThreshold(0.0);
    m_pivotMotor.getConfigurator().apply(softLimits);
  }

  /**
   * Create a command that homes the pivot by driving into the hard stop.
   *
   * <p>The motor drives slowly in the negative direction. When the stator current exceeds the
   * threshold for enough consecutive cycles, the motor is at the hard stop. The encoder is then
   * zeroed, and software limits are enabled.
   *
   * <p>Uses hard-stop zeroing approach (drive into stop, detect stall via current spike).
   *
   * @return a command that completes when homing is done
   */
  public Command homeCommand() {
    final int[] stallCounter = {0};

    return Commands.sequence(
            // Reset state
            Commands.runOnce(() -> {
              m_isHomed = false;
              stallCounter[0] = 0;
            }),
            // Drive into hard stop
            run(() -> {
                  m_pivotMotor.setControl(
                      m_dutyCycleRequest.withOutput(ShooterPivotConstants.HOMING_SPEED));

                  double statorCurrent = m_pivotMotor.getStatorCurrent().getValueAsDouble();
                  if (statorCurrent > ShooterPivotConstants.HOMING_CURRENT_THRESHOLD) {
                    stallCounter[0]++;
                  } else {
                    stallCounter[0] = 0;
                  }
                })
                .until(() -> stallCounter[0] >= ShooterPivotConstants.HOMING_STALL_CYCLES),
            // Zero encoder & enable limits
            Commands.runOnce(() -> {
              m_pivotMotor.setPosition(0);
              m_isHomed = true;
              enableSoftwareLimits();
              m_targetAngleDegrees = ShooterPivotConstants.MIN_ANGLE_DEGREES;
            }),
            // Stop motor
            Commands.runOnce(this::stop))
        .withName("ShooterPivot Home");
  }

  // ==================== COMMANDS ====================

  /**
   * Command to continuously track a target angle from a supplier. This is the main auto-aim command
   * used during shooting.
   *
   * @param angleSupplier supplier that provides the target angle in degrees
   * @return a command that continuously sets the pivot angle
   */
  public Command trackAngleCommand(DoubleSupplier angleSupplier) {
    return run(() -> setAngle(angleSupplier.getAsDouble()))
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot Track Angle");
  }

  /**
   * Command to go to a fixed angle and hold it.
   *
   * @param angleDegrees the target angle in degrees
   * @return a command that holds the angle until cancelled
   */
  public Command goToAngleCommand(double angleDegrees) {
    return run(() -> setAngle(angleDegrees))
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot GoTo " + angleDegrees + "deg");
  }

  /**
   * Manual operator control fallback (duty cycle based). Retained for emergency manual override.
   *
   * @param axisSupplier joystick axis supplier (-1 to 1)
   * @return a command for manual control
   */
  public Command manualControlCommand(DoubleSupplier axisSupplier) {
    return run(() -> {
          double raw = axisSupplier.getAsDouble();
          double deadbanded = MathUtil.applyDeadband(raw, ShooterPivotConstants.MANUAL_DEADBAND);
          setOutput(deadbanded * ShooterPivotConstants.MANUAL_MAX_OUTPUT);
        })
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot Manual");
  }

  public Command zeroEncoderCommand() {
    return runOnce(() -> m_pivotMotor.setPosition(0)).withName("ShooterPivot Zero Encoder");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterPivot/AngleDegrees", getCurrentAngleDegrees());
    SmartDashboard.putNumber("ShooterPivot/TargetAngleDegrees", m_targetAngleDegrees);
    SmartDashboard.putNumber("ShooterPivot/Position (rot)", getPosition());
    SmartDashboard.putNumber("ShooterPivot/Velocity (rps)", getVelocity());
    SmartDashboard.putBoolean("ShooterPivot/IsHomed", m_isHomed);
    SmartDashboard.putBoolean("ShooterPivot/AtTarget", isAtTarget());
    SmartDashboard.putNumber(
        "ShooterPivot/SupplyCurrent", m_pivotMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "ShooterPivot/StatorCurrent", m_pivotMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "ShooterPivot/MotorVoltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "ShooterPivot/DutyCycle", m_pivotMotor.getDutyCycle().getValueAsDouble());
  }
}
