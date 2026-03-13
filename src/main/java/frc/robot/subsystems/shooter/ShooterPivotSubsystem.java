// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal;
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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Shooter pivot subsystem with closed-loop MotionMagic position control.
 *
 * <p>Features: - MotionMagicVoltage for smooth profiled positioning - Gravity feedforward (kG *
 * cos) to hold position against gravity - Hard-stop homing routine to calibrate the integrated
 * encoder - Software limits to protect the mechanism - Manual override fallback for operator
 * control
 */
public class ShooterPivotSubsystem extends SubsystemBase {

  private final TalonFX m_pivotMotor = new TalonFX(ShooterPivotConstants.MOTOR_ID);

  // Control requests
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0);
  private final NeutralOut m_neutralRequest = new NeutralOut();
  StatusSignal<Angle> positionSignal = m_pivotMotor.getPosition();

  // State tracking
  private boolean m_isHomed = true;
  private Angle m_targetAngleDegrees = ShooterPivotConstants.MIN_ANGLE;

  // Trench auto-lower
  private final Supplier<Pose2d> m_poseSupplier;
  private boolean m_trenchMode = false;

  public ShooterPivotSubsystem(Supplier<Pose2d> poseSupplier) {
    m_poseSupplier = poseSupplier;
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    config.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(ShooterPivotConstants.SUPPLY_CURRENT_LIMIT)
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
            ShooterPivotConstants.MAX_ANGLE.minus(ShooterPivotConstants.MIN_ANGLE)));

    m_pivotMotor.getConfigurator().apply(config);
  }

  // ==================== POSITION CONTROL ====================

  // ==================== TRENCH ZONE DETECTION ====================

  /**
   * Check if the robot is in or approaching the trench zone, with hysteresis.
   *
   * <p>Uses wider approach thresholds to ENTER trench mode and tighter thresholds to EXIT, which
   * prevents oscillation at the boundary.
   */
  public boolean isInTrenchZone() {
    // Yeah... we have a bug in here.
    if (m_poseSupplier == null) return false;
    Pose2d pose = m_poseSupplier.get();
    if (pose == null) return false;

    Distance x = Meters.of(pose.getX());
    Distance y = Meters.of(pose.getY());
    Distance fieldW = ShooterPivotConstants.FIELD_WIDTH_METERS;
    Distance margin = ShooterPivotConstants.TRENCH_APPROACH_MARGIN;

    if (m_trenchMode) {
      // Exit thresholds (actual trench zone = tighter)
      boolean inX =
          x.gte(ShooterPivotConstants.TRENCH_X_MIN) && x.lte(ShooterPivotConstants.TRENCH_X_MAX);
      boolean inY = y.lte(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD)
          || y.gte(fieldW.minus(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD));
      return inX && inY;
    } else {
      // Entry thresholds (approach zone = wider by margin)
      boolean inX = x.gte(ShooterPivotConstants.TRENCH_X_MIN.minus(margin))
          && x.lte(ShooterPivotConstants.TRENCH_X_MAX.plus(margin));
      boolean inY = y.lte(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD.plus(margin))
          || y.gte(fieldW.minus(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD).minus(margin));
      return inX && inY;
    }
  }

  public boolean isInTrenchMode() {
    return m_trenchMode;
  }

  // ==================== POSITION CONTROL ====================

  /**
   * Command the pivot to a specific angle using MotionMagic.
   *
   * <p>When the robot is in the trench zone, the angle is capped at TRENCH_LOWER_ANGLE regardless
   * of the requested target. This acts as a safety interlock so no command can accidentally raise
   * the pivot into the trench beam.
   *
   * @param angle target angle (60-80deg range)
   */
  private void setAngle(Angle angle) {
    // Clamp to safe range
    m_targetAngleDegrees =
        Constants.clamp(angle, ShooterPivotConstants.MIN_ANGLE, ShooterPivotConstants.MAX_ANGLE);
    if (!m_trenchMode) {
      setAngleInternal(m_targetAngleDegrees);
    }
  }

  private void setAngleInternal(Angle angle) {
    m_pivotMotor.setControl(
        m_motionMagicRequest.withPosition(ShooterPivotConstants.degreesToMotorRotations(
            angle.minus(ShooterPivotConstants.MIN_ANGLE))));
  }

  /**
   * Get the current pivot angle.
   *
   * @return pivot angle (relative to hard stop zero)
   */
  public Angle getCurrentAngle() {
    return ShooterPivotConstants.motorRotationsToDegrees(positionSignal.getValue())
        .plus(ShooterPivotConstants.MIN_ANGLE);
  }

  /**
   * Check if the pivot is at the target angle within shooting tolerance.
   *
   * @param targetDegrees the target angle
   * @return true if within tolerance
   */
  public boolean isAtAngle(Angle targetDegrees) {
    return getCurrentAngle().isNear(targetDegrees, ShooterPivotConstants.SHOOTING_TOLERANCE);
  }

  /** Check if the pivot is at its current target within shooting tolerance. */
  public boolean isAtTarget() {
    return isAtAngle(m_targetAngleDegrees);
  }

  /** @return whether the pivot has been homed via hard-stop detection. */
  public boolean isHomed() {
    return m_isHomed;
  }

  /** @return the current target angle. */
  public Angle getTargetAngleDegrees() {
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
            ShooterPivotConstants.MAX_ANGLE.minus(ShooterPivotConstants.MIN_ANGLE)));
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

                  Current statorCurrent = m_pivotMotor.getStatorCurrent().getValue();
                  if (statorCurrent.gt(ShooterPivotConstants.HOMING_CURRENT_THRESHOLD)) {
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
              m_targetAngleDegrees = ShooterPivotConstants.MIN_ANGLE;
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
   * @param angleSupplier supplier that provides the target angle
   * @return a command that continuously sets the pivot angle
   */
  public Command trackAngleCommand(Supplier<Angle> angleSupplier) {
    return run(() -> setAngle(angleSupplier.get()))
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot Track Angle");
  }

  /**
   * Command to go to a fixed angle and hold it.
   *
   * @param angleDegrees the target angle
   * @return a command that holds the angle until cancelled
   */
  public Command goToAngleCommand(Angle angleDegrees) {
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
    // Trench safety: actively lower pivot when entering trench zone
    if (isInTrenchZone()) {
      if (!m_trenchMode) {
        if (m_targetAngleDegrees.gt(ShooterPivotConstants.TRENCH_LOWER_ANGLE)) {
          setAngleInternal(ShooterPivotConstants.TRENCH_LOWER_ANGLE);
        }
        m_trenchMode = true;
      }
    } else if (m_trenchMode) {
      setAngleInternal(m_targetAngleDegrees);
      m_trenchMode = false;
    }

    SmartDashboard.putBoolean("ShooterPivot/isInTrenchZone", isInTrenchZone());
    SmartDashboard.putNumber("ShooterPivot/AngleDegrees", getCurrentAngle().in(Degrees));
    SmartDashboard.putNumber("ShooterPivot/TargetAngleDegrees", m_targetAngleDegrees.in(Degrees));
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
    SmartDashboard.putBoolean("ShooterPivot/TrenchMode", m_trenchMode);
  }
}
