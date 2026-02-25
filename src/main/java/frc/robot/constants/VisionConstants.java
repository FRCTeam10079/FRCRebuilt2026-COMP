package frc.robot.constants;

public class VisionConstants {
  public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
  public static final String LIMELIGHT_RIGHT_NAME = "limelight-right";
  public static final String[] LIMELIGHT_NAMES = {LIMELIGHT_LEFT_NAME, LIMELIGHT_RIGHT_NAME};

  public static final int PIPELINE_APRILTAG = 0;

  // Field boundary check margin (meters). 6328 uses 0.5m.
  public static final double FIELD_BORDER_MARGIN = 0.5;
  public static final double FIELD_LENGTH_METERS = 16.54175;
  public static final double FIELD_WIDTH_METERS = 8.0137;

  // Heading divergence gate for vision acceptance.
  // 5deg was too tight - gyro drift causes all vision to be rejected,
  // creating a death spiral where pose diverges further.
  public static final double HEADING_DIVERGENCE_THRESHOLD_DEG = 15.0;

  // Max average tag distance for acceptance. Official LL docs say MT2
  // provides excellent single-tag results at any distance; 3m was too
  // restrictive.
  public static final double MAX_TAG_DISTANCE_METERS = 5.0;

  // Angular velocity rejection threshold (deg/sec).
  // Official Limelight example uses 360deg/s.
  public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 360.0;

  // ==================== STANDARD DEVIATION MODEL ====================
  // XY std dev scales QUADRATICALLY with distance and inversely with tag count,
  // giving much better pose stability.
  //
  // Formula: xyStdDev = XY_STDDEV_COEFFICIENT * (avgDist^2) / tagCount
  //
  // We use 0.005 which yields: 1m away = 0.005, 2m away = 0.02, 3m away = 0.045,
  // 5m away = 0.125
  // These are much tighter than our old flat 0.7, meaning vision is trusted more.

  /** XY std dev coefficient. Multiplied by dist^2 / tagCount for final value. */
  public static final double XY_STDDEV_COEFFICIENT = 0.005;

  /** Floor for XY std dev to prevent overconfidence at very close range. */
  public static final double XY_STDDEV_FLOOR = 0.02;

  /**
   * Theta std dev for multi-tag MT2 observations (radians). Multi-tag MT2 heading is very reliable
   * - trust it to gently correct gyro drift. We use a flat value for MT2 multi-tag since MT2 is
   * gyro-seeded and ambiguity-free. ~0.5 rad (~29 deg) means weak but present heading correction.
   */
  public static final double MULTI_TAG_THETA_STDDEV = 0.5;

  /**
   * Theta std dev for single-tag observations (effectively infinite - don't trust). Single-tag
   * heading is ambiguity-prone and unreliable.
   */
  public static final double SINGLE_TAG_THETA_STDDEV = 9999999.0;

  /**
   * Max pose-difference (meters) between vision and odometry for acceptance. reject if vision says
   * we're suddenly 3+ meters from where odometry thinks we are. Prevents catastrophic pose jumps.
   */
  public static final double MAX_POSE_DIFFERENCE_METERS = 3.0;

  /**
   * Minimum average tag area for MT2 single-tag acceptance. Tags that are too small (far away) have
   * unreliable pose solutions. avgTagArea is 0-100% of image.
   */
  public static final double MIN_TAG_AREA_SINGLE_TAG = 0.003;

  // MT1 pose ambiguity rejection threshold (per-fiducial).
  // 6328 uses 0.4; higher = more ambiguous = less trustworthy.
  public static final double MT1_AMBIGUITY_THRESHOLD = 0.4;

  public static final double MT1_HEADING_CORRECTION_THRESHOLD_DEG = 10.0;
  public static final boolean USE_MT1_HEADING_CORRECTION_WHILE_DISABLED = true;

  protected VisionConstants() {}
}
