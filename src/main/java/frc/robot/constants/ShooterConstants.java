package frc.robot.constants;

public class ShooterConstants {
  public static final int MASTER_MOTOR_ID = 7;
  public static final int SLAVE_MOTOR_ID = 20;

  public static final double SHOOTER_IDLE_RPM = 0;
  public static final double SHOOTER_SPINUP_RPM = 2200;
  public static final double SHOOTER_MAX_RPM = 5500;

  public static final double FENDER_SHOT_RPM = 1800.0;
  public static final double FENDER_SHOT_PIVOT_DEGREES = 78.0;

  public static final double SHOOTER_RPM_TOLERANCE = 150;
  public static final int STABILITY_CYCLES_REQUIRED = 5;
  public static final double ON_TARGET_RPM_PERCENT = 0.04;

  public static final double SHOOTER_KS = 0.15;
  public static final double SHOOTER_KV = 0.12;
  public static final double SHOOTER_KP = 0.5;
  public static final double SHOOTER_KI = 0.0;
  public static final double SHOOTER_KD = 0.0;

  public static final double FEEDER_SPEED = 1.0;
  public static final double SHOOT_FEED_TIMEOUT = 1.0;

  public static final double AUTO_SHOOT_TIMEOUT = 3.0;

  public static final double HEADING_TOLERANCE_DEGREES = 3.0;

  protected ShooterConstants() {
  }
}
