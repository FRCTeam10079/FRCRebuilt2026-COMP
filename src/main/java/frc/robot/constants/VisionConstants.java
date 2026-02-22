package frc.robot.constants;

public class VisionConstants {
  public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
  public static final String LIMELIGHT_RIGHT_NAME = "limelight-right";
  public static final String[] LIMELIGHT_NAMES = {LIMELIGHT_LEFT_NAME, LIMELIGHT_RIGHT_NAME};

  public static final int PIPELINE_APRILTAG = 0;

  public static final double FIELD_BORDER_MARGIN = 0.15;
  public static final double FIELD_LENGTH_METERS = 16.54175;
  public static final double FIELD_WIDTH_METERS = 8.0137;

  public static final double HEADING_DIVERGENCE_THRESHOLD_DEG = 5.0;
  public static final double MAX_TAG_DISTANCE_METERS = 3.0;
  public static final double DEFAULT_XY_STDDEV = 0.3;
  public static final double THETA_STDDEV = 9999999.0;

  public static final double MT1_HEADING_CORRECTION_THRESHOLD_DEG = 10.0;
  public static final boolean USE_MT1_HEADING_CORRECTION_WHILE_DISABLED = true;

  protected VisionConstants() {}
}
