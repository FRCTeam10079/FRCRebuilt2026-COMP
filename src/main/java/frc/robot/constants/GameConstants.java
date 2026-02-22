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

  public static final Translation2d RED_HUB_CENTER = new Translation2d(11.915, 4.035);
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.625, 4.035);

  public static final double HUB_OPENING_HEIGHT_METERS = 72.0 * 0.0254; // 1.8288 m

  public static final double SHOOTER_RELEASE_HEIGHT_METERS = 19.5 * 0.0254; // 0.4953 m

  public static final double SHOT_HEIGHT_DELTA_METERS = HUB_OPENING_HEIGHT_METERS - SHOOTER_RELEASE_HEIGHT_METERS;

  protected GameConstants() {
  }
}
