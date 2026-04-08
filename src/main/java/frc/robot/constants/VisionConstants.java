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

  public static final double MT1_HEADING_CORRECTION_THRESHOLD_DEG = 10.0;
  public static final boolean USE_MT1_HEADING_CORRECTION_WHILE_DISABLED = true;

  // One-shot heading bootstrap: fires once on first multi-tag result if
  // MT1 heading diverges from gyro by more than this threshold.
  public static final double HEADING_BOOTSTRAP_THRESHOLD_DEG = 30.0;

  // ==================== JITTER MITIGATION ====================
  // Single-tag stddev multiplier: single-tag MT1 is inherently noisier due to
  // ambiguity. This multiplier inflates the XY stddev for single-tag results
  // so the Kalman filter trusts them less. Log data showed single-tag
  // observations causing 5-13cm oscillations per frame.
  public static final double SINGLE_TAG_STDDEV_MULTIPLIER = 3.0;

  // Ambiguity-scaled stddev: instead of a binary accept/reject at 0.4,
  // we also scale the stddev by (1 + ambiguity * this factor).
  // An observation with ambiguity=0.35 gets stddev multiplied by ~2.05x.
  // This makes borderline-ambiguity observations less trusted rather than
  // fully trusted or fully rejected.
  public static final double AMBIGUITY_STDDEV_SCALE = 3.0;

  // Pose jump rejection: if a vision pose is more than this distance (meters)
  // from the current odometry pose, reject it outright. Log data showed a
  // 4.76m jump from a bogus single-tag observation that still passed all checks.
  public static final double MAX_POSE_JUMP_METERS = 0.75;

  protected VisionConstants() {}
}
