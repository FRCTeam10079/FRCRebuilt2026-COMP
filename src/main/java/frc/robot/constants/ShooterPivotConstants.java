package frc.robot.constants;

public class ShooterPivotConstants {
  public static final int MOTOR_ID = 23;

  public static final double GEAR_RATIO = 118.0;

  public static final double MIN_ANGLE_DEGREES = 60.0;
  public static final double MAX_ANGLE_DEGREES = 80.0;

  public static final double HOMING_SPEED = -0.06;

  public static final double HOMING_CURRENT_THRESHOLD = 20.0;

  public static final int HOMING_STALL_CYCLES = 5;

  public static final double KP = 6.0;
  public static final double KI = 0.0;
  public static final double KD = 0.1;
  public static final double KS = 0.18;
  public static final double KV = 0.12;

  public static final double KG = 0.3;

  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 40.0;
  public static final double MOTION_MAGIC_ACCELERATION = 80.0;
  public static final double MOTION_MAGIC_JERK = 400.0;

  public static final double POSITION_TOLERANCE_DEGREES = 1.0;

  public static final double SHOOTING_TOLERANCE_DEGREES = 2.0;

  public static final int SUPPLY_CURRENT_LIMIT = 30;
  public static final int STATOR_CURRENT_LIMIT = 60;

  public static final double MANUAL_MAX_OUTPUT = 0.35;
  public static final double MANUAL_DEADBAND = 0.1;

  public static double degreesToMotorRotations(double degrees) {
    return degrees * GEAR_RATIO / 360.0;
  }

  public static double motorRotationsToDegrees(double motorRotations) {
    return motorRotations * 360.0 / GEAR_RATIO;
  }

  protected ShooterPivotConstants() {
  }
}
