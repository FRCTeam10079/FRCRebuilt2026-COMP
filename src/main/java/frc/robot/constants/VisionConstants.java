package frc.robot.constants;

/**
 * Vision constants for dual-Limelight MegaTag2-only pose estimation.
 *
 * Filtering approach:
 * - MT2 only - no MegaTag1 fallback (removes noise from ambiguous single-tag
 * solves)
 * - Tight 5deg heading divergence gate (gyro vs vision heading sanity check)
 * - Hard 3m max tag distance cutoff
 * - Cubic std dev scaling for single tags, linear for multi-tag
 * - Never trust Limelight rotation (theta stddev = 9999999)
 * - 0.15m field border margin (tight out-of-bounds rejection)
 * - 50ms transmission delay subtracted from timestamps for accurate Kalman
 * filter fusion
 */
public class VisionConstants {
  // ==================== LIMELIGHT NAMES ====================
  public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
  public static final String LIMELIGHT_RIGHT_NAME = "limelight-right";
  public static final String[] LIMELIGHT_NAMES = { LIMELIGHT_LEFT_NAME, LIMELIGHT_RIGHT_NAME };

  // ==================== PIPELINE ====================
  public static final int PIPELINE_APRILTAG = 0;

  // ==================== FIELD DIMENSIONS ====================
  public static final double FIELD_BORDER_MARGIN = 0.15;
  public static final double FIELD_LENGTH_METERS = 16.54175;
  public static final double FIELD_WIDTH_METERS = 8.0137;

  // ==================== REJECTION THRESHOLDS ====================
  /**
   * Max tag distance (meters) - reject MT2 measurements from tags farther than
   * this.
   */
  public static final double MAX_TAG_DISTANCE = 3.0;

  /**
   * Heading divergence threshold (degrees). If the MT2 pose heading disagrees
   * with the
   * pose estimator heading by more than this, reject the measurement. Catches bad
   * heading
   * feedback loops.
   */
  public static final double HEADING_DIVERGENCE_THRESHOLD_DEG = 5.0;

  // ==================== STANDARD DEVIATIONS ====================
  /**
   * Base XY standard deviation for MT2 estimates. Scaled by distance:
   * - 1 tag: base * d^3 (cubic - heavily penalizes distant single-tag solves)
   * - 2+ tags: base * d (linear - multi-tag geometry is much more reliable)
   */
  public static final double MT2_BASE_XY_STDDEV = 0.3;

  /**
   * Rotation std dev - effectively infinite so the Kalman filter ignores
   * Limelight heading.
   */
  public static final double ROTATION_STDDEV = 9999999.0;

  // ==================== LATENCY ====================
  /**
   * Additional network transmission delay (seconds) subtracted from timestamps.
   * Accounts for the time between Limelight capture and the pose reaching the
   * roboRIO
   * via NetworkTables.
   */
  public static final double LIMELIGHT_TRANSMISSION_DELAY = 0.05;

  protected VisionConstants() {
  }
}
