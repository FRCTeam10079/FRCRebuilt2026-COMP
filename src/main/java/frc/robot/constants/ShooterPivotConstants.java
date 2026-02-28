package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;

public class ShooterPivotConstants {
  public static final int MOTOR_ID = 23;

  // ==================== GEAR RATIO & RANGE ====================
  /** Motor-to-pivot gear ratio (motor rotations per pivot rotation). */
  public static final double GEAR_RATIO = 118.0;

  /** Physical range of motion in degrees. */
  public static final Angle MIN_ANGLE = Degrees.of(60.0);

  public static final Angle MAX_ANGLE = Degrees.of(80.0);

  // ==================== HOMING ====================
  /** Duty cycle output for slow hard-stop homing (negative = toward hard stop). */
  public static final double HOMING_SPEED = -0.06;

  /** Stator current threshold (amps) to detect the hard stop. */
  public static final Current HOMING_CURRENT_THRESHOLD = Amps.of(20.0);

  /** Number of consecutive loops above threshold to confirm hard stop. */
  public static final int HOMING_STALL_CYCLES = 5;

  // ==================== CLOSED-LOOP PID (Slot 0) ====================
  // These values MUST be tuned on the actual robot!
  public static final double KP = 6.0;
  public static final double KI = 0.0;
  public static final double KD = 0.1;
  public static final double KS = 0.18;
  public static final double KV = 0.12;

  /**
   * Gravity feedforward coefficient. Applied as kG * cos(pivotAngle) to hold position against
   * gravity. Tune by finding the minimum voltage to hold the pivot at various angles.
   */
  public static final double KG = 0.3;

  // ==================== MOTION MAGIC ====================
  /** Cruise velocity in motor rotations per second. */
  public static final AngularVelocity MOTION_MAGIC_CRUISE_VELOCITY = RotationsPerSecond.of(40.0);
  /** Acceleration in motor rotations per second^2. */
  public static final AngularAcceleration MOTION_MAGIC_ACCELERATION =
      RotationsPerSecondPerSecond.of(80.0);
  /** Jerk in motor rotations per second^3 (0 = trapezoidal, >0 = S-curve). */
  public static final Velocity<AngularAccelerationUnit> MOTION_MAGIC_JERK =
      RotationsPerSecondPerSecond.per(Second).of(400.0);

  // ==================== TOLERANCES ====================

  /** Wider tolerance for "ready to shoot" in degrees. */
  public static final Angle SHOOTING_TOLERANCE = Degrees.of(2.0);

  // ==================== CURRENT LIMITS ====================
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);

  // ==================== MANUAL OVERRIDE ====================
  /** Maximum duty cycle for manual operator control (fallback). */
  public static final double MANUAL_MAX_OUTPUT = 0.35;

  public static final double MANUAL_DEADBAND = 0.1;

  // ==================== CONVERSIONS ====================
  /** Convert position of pivot angle to motor rotations. motorRotations = position * GEAR_RATIO */
  public static Angle degreesToMotorRotations(Angle position) {
    return position.times(GEAR_RATIO);
  }

  /** Convert motor rotations to degrees of pivot angle. degrees = motorPosition / GEAR_RATIO */
  public static Angle motorRotationsToDegrees(Angle motorPosition) {
    return motorPosition.div(GEAR_RATIO);
  }

  protected ShooterPivotConstants() {}
}
