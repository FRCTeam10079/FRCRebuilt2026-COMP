package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AprilTagMaps;

/**
 * Constants for the pathfinding system.
 *
 * <p>Robot physics calculations for 2026 REBUILT:
 *
 * <p>Robot dimensions: 33" x 34" with bumpers Diagonal: √(33² + 34²) = √2245 ≈ 47.4" → radius ≈
 * 0.602m Inflation radius: 0.65m (3.25 grid cells) provides ~5cm buffer for safe rotation
 */
public final class PathfindingConstants {

  private PathfindingConstants() {}

  // ==================== ROBOT PHYSICS ====================
  /** Robot width with bumpers in inches */
  public static final double ROBOT_WIDTH_INCHES = 31.5;

  /** Robot length with bumpers in inches */
  public static final double ROBOT_LENGTH_INCHES = 36.5;

  /** Robot diagonal (worst-case rotation footprint) in meters */
  public static final double ROBOT_DIAGONAL_METERS = Units.inchesToMeters(Math.sqrt(
      ROBOT_WIDTH_INCHES * ROBOT_WIDTH_INCHES + ROBOT_LENGTH_INCHES * ROBOT_LENGTH_INCHES));

  /**
   * Inflation radius for pathfinding obstacles. This ensures the robot can safely rotate near
   * obstacles without clipping. Calculated as half-diagonal + safety buffer.
   */
  public static final double INFLATION_RADIUS_METERS = 0.65;

  // ==================== GRID CONFIGURATION ====================
  /** Grid node size in meters */
  public static final double NODE_SIZE_METERS = 0.2;

  /** Number of grid cells to inflate obstacles by */
  public static final int INFLATION_CELLS =
      (int) Math.ceil(INFLATION_RADIUS_METERS / NODE_SIZE_METERS);

  /** Field length in meters */
  public static final double FIELD_LENGTH_METERS = 16.54;

  /** Field width in meters */
  public static final double FIELD_WIDTH_METERS = 8.07;

  // ==================== PATH CONSTRAINTS ====================
  /** Maximum velocity during pathfinding (m/s) */
  public static final double MAX_VELOCITY_MPS = 4.0;

  /** Maximum acceleration during pathfinding (m/s²) */
  public static final double MAX_ACCELERATION_MPSS = 3.0;

  /** Maximum angular velocity during pathfinding (rad/s) */
  public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.PI * 1.5;

  /** Maximum angular acceleration during pathfinding (rad/s²) */
  public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = Math.PI * 2.0;

  // ==================== SCORING POSE OFFSETS ====================
  /**
   * Standoff distance from AprilTag when scoring (meters). Robot stops this far back from the tag.
   */
  public static final double SCORING_STANDOFF_METERS = 0.6;

  // ==================== TOLERANCES ====================
  /** Position tolerance for considering the robot at the goal (meters) */
  public static final double POSITION_TOLERANCE_METERS = 0.05;

  /** Rotation tolerance for considering the robot at the goal (radians) */
  public static final double ROTATION_TOLERANCE_RADIANS = Math.toRadians(3.0);

  // ==================== APRILTAG TARGET HELPER ====================

  /**
   * Calculate the scoring pose offset from an AprilTag.
   *
   * @param tagId The AprilTag ID to target
   * @return The scoring pose (standoff position facing the tag)
   */
  public static Pose2d getScoringPoseForTag(int tagId) {
    double[] tagData = AprilTagMaps.aprilTagMap.get(tagId);
    if (tagData == null) {
      throw new IllegalArgumentException("Unknown AprilTag ID: " + tagId);
    }

    // Convert tag position from inches to meters
    double tagX = Units.inchesToMeters(tagData[0]);
    double tagY = Units.inchesToMeters(tagData[1]);
    double tagYawDegrees = tagData[3];

    // Tag faces outward, robot needs to face toward the tag
    Rotation2d tagFacing = Rotation2d.fromDegrees(tagYawDegrees);
    Rotation2d robotFacing = tagFacing.plus(Rotation2d.k180deg);

    // Calculate standoff position (move back from tag along its facing direction)
    Translation2d tagPosition = new Translation2d(tagX, tagY);
    Translation2d standoffOffset =
        new Translation2d(SCORING_STANDOFF_METERS, 0).rotateBy(tagFacing);
    Translation2d scoringPosition = tagPosition.plus(standoffOffset);

    // === DEBUG LOGGING ===
    System.out.println("\n========== APRILTAG POSE CALCULATION DEBUG ==========");
    System.out.println("[TagPose] Tag ID: " + tagId);
    System.out.println("[TagPose] Raw Tag Data (inches): X=" + tagData[0] + ", Y=" + tagData[1]
        + ", Yaw=" + tagYawDegrees + "°");
    System.out.println("[TagPose] Tag Position (meters): X=" + String.format("%.3f", tagX) + ", Y="
        + String.format("%.3f", tagY));
    System.out.println(
        "[TagPose] Tag Facing Direction: " + String.format("%.1f", tagFacing.getDegrees()) + "°");
    System.out.println(
        "[TagPose] Robot Should Face: " + String.format("%.1f", robotFacing.getDegrees()) + "°");
    System.out.println("[TagPose] Standoff Distance: " + SCORING_STANDOFF_METERS + "m");
    System.out.println(
        "[TagPose] Standoff Offset Vector: X=" + String.format("%.3f", standoffOffset.getX())
            + ", Y=" + String.format("%.3f", standoffOffset.getY()));
    System.out.println(
        "[TagPose] Final Scoring Position: X=" + String.format("%.3f", scoringPosition.getX())
            + ", Y=" + String.format("%.3f", scoringPosition.getY()));
    System.out.println("[TagPose] Final Pose: (" + String.format("%.3f", scoringPosition.getX())
        + ", " + String.format("%.3f", scoringPosition.getY()) + ", "
        + String.format("%.1f", robotFacing.getDegrees()) + "°)");
    System.out.println("=====================================================\n");

    return new Pose2d(scoringPosition, robotFacing);
  }

  /**
   * Get the pose of AprilTag 10 (Red Alliance Hub Face) with scoring standoff.
   *
   * @return Scoring pose for AprilTag 10
   */
  public static Pose2d getAprilTag10ScoringPose() {
    return getScoringPoseForTag(10);
  }
}
