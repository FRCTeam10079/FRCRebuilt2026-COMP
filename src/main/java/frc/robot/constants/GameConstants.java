package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class GameConstants {
  public static final double AUTO_DURATION = 20.0;
  public static final double TELEOP_DURATION = 150.0;
  public static final double ENDGAME_THRESHOLD = 30.0;
  public static final double CRITICAL_TIME_THRESHOLD = 10.0;

  public static final double TRANSITION_START_TIME = 150.0;
  public static final double TRANSITION_END_TIME = 140.0;
  public static final double TRANSITION_DURATION = 10.0;

  public static final int PRELOAD_FUEL_LIMIT = 8;

  public static final int CLIMB_L1_POINTS = 15;
  public static final int CLIMB_L2_POINTS = 30;
  public static final int CLIMB_L3_POINTS = 50;

  public static final double MAX_HEIGHT_DURING_CLIMB = 30.0;

  // ==================== HUB GEOMETRY ====================

  /**
   * Hub center positions derived from AprilTag locations.
   *
   * <p>Red hub: tags 9 & 10; center ~ (469.1 in, 158.85 in) -> (11.915 m, 4.035 m) Blue hub: tags
   * 25 & 26; center ~ (182.1 in, 158.85 in) -> (4.625 m, 4.035 m)
   *
   * <p>These are used by ShooterMath to compute distance and heading to the active hub for
   * distance-based shooting.
   */
  public static final Translation2d RED_HUB_CENTER = new Translation2d(11.915, 4.035);

  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.625, 4.035);

  /** Height of the hub opening from the field carpet, in meters. (72 inches) */
  public static final double HUB_OPENING_HEIGHT_METERS = 72.0 * 0.0254; // 1.8288 m

  /** Height at which the shooter releases the ball, in meters. (19.5 inches) */
  public static final double SHOOTER_RELEASE_HEIGHT_METERS = 19.5 * 0.0254; // 0.4953 m

  /** Vertical distance the ball must travel, in meters. */
  public static final double SHOT_HEIGHT_DELTA_METERS =
      HUB_OPENING_HEIGHT_METERS - SHOOTER_RELEASE_HEIGHT_METERS;

  protected GameConstants() {}
}
