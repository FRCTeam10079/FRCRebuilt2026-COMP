package frc.robot.constants;

public class VisionConstants {
  public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
  public static final String LIMELIGHT_RIGHT_NAME = "limelight-right";
  public static final String[] LIMELIGHT_NAMES = {LIMELIGHT_LEFT_NAME, LIMELIGHT_RIGHT_NAME};

  public static final int PIPELINE_APRILTAG = 0;

  public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 360.0;

  public static final double FIELD_BORDER_MARGIN = 0.5;
  public static final double FIELD_LENGTH_METERS = 16.54;
  public static final double FIELD_WIDTH_METERS = 8.07;

  public static final double MT2_BASE_XY_STDDEV = 0.3;
  public static final double MT2_THETA_STDDEV = 99999.0;

  public static final double MT1_MULTI_TAG_XY_STDDEV = 0.7;
  public static final double MT1_MULTI_TAG_THETA_STDDEV = 0.5;

  public static final double MT1_SINGLE_TAG_XY_STDDEV = 1.5;
  public static final double MT1_SINGLE_TAG_THETA_STDDEV = 99999.0;

  public static final double HEADING_DIVERGENCE_THRESHOLD_DEG = 30.0;
  public static final double MT1_HEADING_CORRECTION_THRESHOLD_DEG = 10.0;

  public static final boolean USE_MT1_HEADING_CORRECTION_WHILE_DISABLED = true;

  public static final double POSE_DIVERGENCE_WARNING_METERS = 2.0;

  protected VisionConstants() {}
}
