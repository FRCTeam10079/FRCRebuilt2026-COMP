package frc.robot.constants;

public class VisionConstants {
  public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
  public static final String LIMELIGHT_RIGHT_NAME = "limelight-right";
  public static final String[] LIMELIGHT_NAMES = {LIMELIGHT_LEFT_NAME, LIMELIGHT_RIGHT_NAME};

  public static final int PIPELINE_APRILTAG = 0;

  // Field boundary check margin (meters). 6328 uses 0.5m.
  public static final double FIELD_BORDER_MARGIN = 0.5;
  // Official 2026 REBUILT field dimensions from WPILib 2026-rebuilt-welded.json
  public static final double FIELD_LENGTH_METERS = 16.541;
  public static final double FIELD_WIDTH_METERS = 8.069;

  // Heading divergence gate for vision acceptance.
  // 5deg was too tight - gyro drift causes all vision to be rejected,
  // creating a death spiral where pose diverges further.
  public static final double HEADING_DIVERGENCE_THRESHOLD_DEG = 45.0;

  // Angular velocity rejection threshold (deg/sec).
  // Official Limelight example uses 360deg/s.
  public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 360.0;

  // ==================== STANDARD DEVIATION MODEL
  // ====================
  // Formula: xyStdDev = XY_STDDEV_COEFFICIENT * avgDist^XY_STDDEV_EXPONENT /
  // tagCount^2
  // Formula: thetaStdDev = THETA_STDDEV_COEFFICIENT * avgDist^XY_STDDEV_EXPONENT
  // / tagCount^2
  // (multi-tag only; single-tag theta = POSITIVE_INFINITY)

  /** XY std dev coefficient (6328 uses 0.01). */
  public static final double XY_STDDEV_COEFFICIENT = 0.01;

  /** Distance exponent for std dev scaling */
  public static final double XY_STDDEV_EXPONENT = 1.2;

  /** Theta std dev coefficient for multi-tag */
  public static final double THETA_STDDEV_COEFFICIENT = 0.03;

  // MT1 pose ambiguity rejection threshold (per-fiducial).
  // 6328 uses 0.4; higher = more ambiguous = less trustworthy.
  public static final double MT1_AMBIGUITY_THRESHOLD = 0.4;

  // MT1 heading offset (degrees). If MT1 heading is consistently 180deg off,
  // set this to 180.0.
  public static final double MT1_HEADING_OFFSET_DEG = 180.0;

  public static final double MT1_HEADING_CORRECTION_THRESHOLD_DEG = 10.0;
  public static final boolean USE_MT1_HEADING_CORRECTION_WHILE_DISABLED = true;

  protected VisionConstants() {}
}
