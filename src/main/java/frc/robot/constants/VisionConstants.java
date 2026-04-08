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

  // ==================== STANDARD DEVIATION MODEL ====================
  // Formula: xyStdDev = XY_STDDEV_COEFFICIENT * avgDist^XY_STDDEV_EXPONENT /
  // tagCount^2
  // (multi-tag only for theta; single-tag theta = POSITIVE_INFINITY)

  /** XY std dev coefficient. */
  public static final double XY_STDDEV_COEFFICIENT = 0.01;

  /**
   * Distance exponent for std dev scaling. Raised from 1.2 to 1.5: tags beyond 3m are much less
   * reliable. At 3m: 0.01*3^1.5 = 0.052 vs old 0.037.
   */
  public static final double XY_STDDEV_EXPONENT = 1.5;

  /** Theta std dev coefficient for multi-tag */
  public static final double THETA_STDDEV_COEFFICIENT = 0.03;

  // Pose ambiguity rejection threshold (per-fiducial).
  // Lowered from 0.4 to 0.25: log (40 matches, 236k measurements)
  // showed tags with ambiguity 0.25-0.4 causing 5-13cm frame-to-frame jitter.
  public static final double MT1_AMBIGUITY_THRESHOLD = 0.25;

  public static final double MT1_HEADING_CORRECTION_THRESHOLD_DEG = 10.0;
  public static final boolean USE_MT1_HEADING_CORRECTION_WHILE_DISABLED = true;

  // One-shot heading bootstrap: fires once on first multi-tag result if
  // MT1 heading diverges from gyro by more than this threshold.
  public static final double HEADING_BOOTSTRAP_THRESHOLD_DEG = 30.0;

  // ==================== JITTER MITIGATION ====================
  // Single-tag stddev multiplier: single-tag is inherently noisier.
  // Log: single-tag is 2.5x worse divergence, 3.1x worse jitter.
  // Previous 1.5x was too weak with MT1; with MT2 (no ambiguity problem)
  // 3.0x appropriately down-weights single-tag while still allowing corrections.
  public static final double SINGLE_TAG_STDDEV_MULTIPLIER = 3.0;

  // Ambiguity-scaled stddev: continuously degrade trust as ambiguity rises.
  // At ambiguity=0.25: stddev *= 1.75, at ambiguity=0.1: stddev *= 1.3.
  public static final double AMBIGUITY_STDDEV_SCALE = 3.0;

  // ==================== POSE JUMP HANDLING (GRADUATED TRUST)
  // ====================
  // Hard reject only for clearly impossible poses. Raised from 2.0 to 8.0.
  // The old 2.0m threshold caused death spirals in 22.8% of all vision data
  // across 40 match logs; permanently locking out vision for entire matches.
  // 8.0m is roughly half the field diagonal; anything beyond is physically
  // impossible.
  public static final double MAX_POSE_JUMP_METERS = 8.0;

  // Graduated trust: instead of binary reject, inflate stddev proportionally
  // to vision-odometry divergence. This allows recovery from drift while
  // still down-weighting suspicious measurements.
  // Below this distance (meters), no inflation applied.
  public static final double DIVERGENCE_RAMP_START_METERS = 0.5;

  // Stddev multiplier per meter of divergence above the ramp start.
  // At 2m: multiplier = 1 + 2.0*(2.0-0.5) = 4.0x (gentle correction)
  // At 4m: multiplier = 1 + 2.0*(4.0-0.5) = 8.0x (very slow pull-back)
  // At 6m: multiplier = 1 + 2.0*(6.0-0.5) = 12.0x (barely trusted, but NOT
  // rejected)
  // This is self-correcting: as pose converges, trust automatically increases.
  public static final double DIVERGENCE_STDDEV_SCALE = 2.0;

  // ==================== SINGLE-TAG DISTANCE LIMIT ====================
  // Reject single-tag observations beyond this distance (meters).
  // Log: single-tag at >2.5m with ambiguity 0.2-0.4 causes jitter.
  // 4.0m provides margin while filtering the worst far-field noise.
  public static final double SINGLE_TAG_MAX_DISTANCE_METERS = 4.0;

  protected VisionConstants() {}
}
