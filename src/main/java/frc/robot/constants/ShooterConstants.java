package frc.robot.constants;

public class ShooterConstants {
  public static final int MASTER_MOTOR_ID = 7;
  public static final int SLAVE_MOTOR_ID = 20;

  // ==================== VELOCITY SETPOINTS ====================
  public static final double SHOOTER_IDLE_RPM = 0;
  /** Default fixed spin-up RPM (used when NOT in distance-based mode). */
  public static final double SHOOTER_SPINUP_RPM = 2200;

  public static final double SHOOTER_MAX_RPM = 5500;

  // ==================== FENDER / PRESET SHOTS ====================
  /** RPM for a close-range fender shot (pressed against the hub). */
  public static final double FENDER_SHOT_RPM = 1800.0;
  /** Pivot angle for a close-range fender shot (degrees). */
  public static final double FENDER_SHOT_PIVOT_DEGREES = 78.0;

  // ==================== TOLERANCES ====================
  public static final double SHOOTER_RPM_TOLERANCE = 150;
  public static final int STABILITY_CYCLES_REQUIRED = 5;
  /** Percentage tolerance for on-target check (inspired by 254's 4% tolerance). */
  public static final double ON_TARGET_RPM_PERCENT = 0.04;

  // ==================== PID GAINS (Slot 0) ====================
  public static final double SHOOTER_KS = 0.15;
  public static final double SHOOTER_KV = 0.12;
  public static final double SHOOTER_KP = 0.5;
  public static final double SHOOTER_KI = 0.0;
  public static final double SHOOTER_KD = 0.0;

  // ==================== INDEXER / FEEDER ====================
  public static final double FEEDER_SPEED = 1.0;
  public static final double SHOOT_FEED_TIMEOUT = 1.0;

  // ==================== AUTO SHOOTING ====================
  /** Timeout for auto shoot command before giving up (seconds). */
  public static final double AUTO_SHOOT_TIMEOUT = 3.0;

  // ==================== HEADING ALIGNMENT ====================
  /** Heading tolerance for "aligned to hub" in degrees. */
  public static final double HEADING_TOLERANCE_DEGREES = 3.0;

  protected ShooterConstants() {}
}
