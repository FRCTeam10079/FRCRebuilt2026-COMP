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

  // Base XY standard deviation for vision measurements.
  // Official Limelight example uses 0.7. Scaled by distance for MT2.
  public static final double DEFAULT_XY_STDDEV = 0.7;
  public static final double THETA_STDDEV = 9999999.0;

  // MT1 pose ambiguity rejection threshold (per-fiducial).
  // 6328 uses 0.4; higher = more ambiguous = less trustworthy.
  public static final double MT1_AMBIGUITY_THRESHOLD = 0.4;

  public static final double MT1_HEADING_CORRECTION_THRESHOLD_DEG = 10.0;
  public static final boolean USE_MT1_HEADING_CORRECTION_WHILE_DISABLED = true;

  protected VisionConstants() {}
}
