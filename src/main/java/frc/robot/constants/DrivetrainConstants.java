package frc.robot.constants;

public class DrivetrainConstants {
  public static final double MAX_SPEED_MPS = 10.0;
  public static final double MAX_ANGULAR_RATE_RAD_PER_SEC = Math.PI * 2.0;

  public static final double NORMAL_SPEED_COEFFICIENT = 1.0;
  public static final double SLOW_MODE_COEFFICIENT = 0.7;
  public static final double SCORING_SPEED_COEFFICIENT = 0.5;

  public static final double DEADBAND_PERCENT = 0.1;
  public static final double SKEW_COMPENSATION_SCALAR = -0.03;

  public static final double CHOREO_TRANSLATION_KP = 7.0;
  public static final double CHOREO_TRANSLATION_KI = 0.0;
  public static final double CHOREO_TRANSLATION_KD = 0.0;
  public static final double CHOREO_HEADING_KP = 5.0;
  public static final double CHOREO_HEADING_KI = 0.0;
  public static final double CHOREO_HEADING_KD = 0.0;

  public static final double PP_TRANSLATION_KP = 7.0;
  public static final double PP_TRANSLATION_KI = 0.0;
  public static final double PP_TRANSLATION_KD = 0.0;
  public static final double PP_ROTATION_KP = 5.0;
  public static final double PP_ROTATION_KI = 0.0;
  public static final double PP_ROTATION_KD = 0.0;

  public static final double POSITION_TOLERANCE_METERS = 0.02;
  public static final double YAW_TOLERANCE_RADIANS = Math.PI / 32;

  public static final double ALIGN_PID_KP = 8.0;
  public static final double ALIGN_PID_KI = 0.0;
  public static final double ALIGN_PID_KD = 0.01;
  public static final double ALIGN_ROTATION_KP = 3.0;
  public static final double ALIGN_ROTATION_KD = 0.02;

  public static final double ALIGN_SPEED_MPS = 3.5;
  public static final double ALIGN_ROTATION_SPEED = 0.9;

  public static final double ALIGN_OFFSET_X_LEFT = -0.41;
  public static final double ALIGN_OFFSET_Y_LEFT = 0.13;
  public static final double ALIGN_OFFSET_X_RIGHT = -0.41;
  public static final double ALIGN_OFFSET_Y_RIGHT = -0.23;
  public static final double ALIGN_OFFSET_X_CENTER = -0.50;
  public static final double ALIGN_OFFSET_Y_CENTER = 0.0;

  protected DrivetrainConstants() {}
}
