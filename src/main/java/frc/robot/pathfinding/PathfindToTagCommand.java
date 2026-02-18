package frc.robot.pathfinding;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;
import java.util.function.Supplier;

/**
 * Command that pathfinds to a target pose using the AD* algorithm and follows the path using PID
 * control.
 *
 * <p>This command combines pathfinding logic (AD* search on a navgrid) with motion control pattern
 * (PID-based trajectory following).
 *
 * <p>The command: Sets the pathfinding goal (e.g., AprilTag scoring pose) Waits for the AD*
 * background thread to calculate a path Follows the path using pure pursuit / PID control Ends when
 * the robot reaches the goal within tolerance
 */
public class PathfindToTagCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final PathConstraints constraints;

  // PID controllers for path following
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  // State
  private Pose2d targetPose;
  private List<Translation2d> currentPath;
  private int currentWaypointIndex;
  private final Timer pathWaitTimer = new Timer();
  private boolean hasPath = false;

  // Debug counter for periodic logging
  private int debugCounter = 0;

  // NetworkTables publishers for AdvantageScope visualization
  private final NetworkTable pathfindingTable;
  private final StructPublisher<Pose2d> currentPosePublisher;
  private final StructPublisher<Pose2d> goalPosePublisher;
  private final StructPublisher<Pose2d> targetWaypointPublisher;
  private final StructArrayPublisher<Pose2d> pathPublisher;

  // Field2d for visualization in AdvantageScope / Shuffleboard
  private static final Field2d pathfindingField = new Field2d();

  static {
    // Publish Field2d once so AdvantageScope can see it
    SmartDashboard.putData("Pathfinding/Field", pathfindingField);
  }

  // Pure pursuit parameters
  private static final double LOOKAHEAD_DISTANCE = 0.5; // meters
  private static final double WAYPOINT_TOLERANCE = 0.3; // meters to advance waypoint

  /**
   * Create a command to pathfind to a specific AprilTag.
   *
   * @param drivetrain The swerve drivetrain
   * @param tagId The AprilTag ID to target
   */
  public static PathfindToTagCommand toAprilTag(CommandSwerveDrivetrain drivetrain, int tagId) {
    return new PathfindToTagCommand(
        drivetrain,
        () -> PathfindingConstants.getScoringPoseForTag(tagId),
        PathConstraints.DEFAULT);
  }

  /**
   * Create a command to pathfind to AprilTag 10 (Red Alliance Hub Face).
   *
   * @param drivetrain The swerve drivetrain
   */
  public static PathfindToTagCommand toAprilTag10(CommandSwerveDrivetrain drivetrain) {
    return toAprilTag(drivetrain, 10);
  }

  /**
   * Create a pathfinding command.
   *
   * @param drivetrain The swerve drivetrain
   * @param targetPoseSupplier Supplier for the target pose
   * @param constraints Path following constraints
   */
  public PathfindToTagCommand(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> targetPoseSupplier,
      PathConstraints constraints) {

    this.drivetrain = drivetrain;
    this.targetPoseSupplier = targetPoseSupplier;
    this.constraints = constraints;

    // Translation PID (field-relative)
    this.xController = new PIDController(3.0, 0.0, 0.1);
    this.yController = new PIDController(3.0, 0.0, 0.1);

    // Rotation PID with continuous input
    this.rotationController = new PIDController(3.0, 0.0, 0.1);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    // Initialize NetworkTables publishers for AdvantageScope
    pathfindingTable = NetworkTableInstance.getDefault().getTable("Pathfinding");
    currentPosePublisher =
        pathfindingTable.getStructTopic("CurrentPose", Pose2d.struct).publish();
    goalPosePublisher =
        pathfindingTable.getStructTopic("GoalPose", Pose2d.struct).publish();
    targetWaypointPublisher =
        pathfindingTable.getStructTopic("TargetWaypoint", Pose2d.struct).publish();
    pathPublisher = pathfindingTable.getStructArrayTopic("Path", Pose2d.struct).publish();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Get target pose
    targetPose = targetPoseSupplier.get();

    // Get current robot state
    Pose2d currentPose = drivetrain.getState().Pose;
    ChassisSpeeds currentSpeeds = drivetrain.getState().Speeds;

    // Convert to field-relative velocity
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
    Translation2d velocity =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    // Ensure pathfinder is initialized
    Pathfinding.ensureInitialized();

    // Set the pathfinding problem
    Pathfinding.setProblem(currentPose, targetPose, velocity);

    // Reset state
    currentPath = null;
    currentWaypointIndex = 0;
    hasPath = false;
    pathWaitTimer.restart();

    // Reset PID controllers
    xController.reset();
    yController.reset();
    rotationController.reset();

    System.out.println("[PathfindToTag] Starting pathfind from "
        + formatPose(currentPose)
        + " to "
        + formatPose(targetPose));
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    // === PERIODIC DEBUG (every 50 loops ~ 1 second) ===
    debugCounter++;
    boolean shouldLogDebug = (debugCounter % 50 == 1);

    // Check for new path from background thread
    if (Pathfinding.isNewPathAvailable()) {
      currentPath = Pathfinding.getCurrentPathWaypoints();
      currentWaypointIndex = 0;
      hasPath = currentPath != null && currentPath.size() >= 2;

      if (hasPath) {
        System.out.println(
            "[PathfindToTag] Received path with " + currentPath.size() + " waypoints");
        // Log all waypoints when path is received
        System.out.println("[PathfindToTag] === PATH WAYPOINTS ===");
        for (int i = 0; i < currentPath.size(); i++) {
          Translation2d wp = currentPath.get(i);
          System.out.println("  [" + i + "]: (" + String.format("%.3f", wp.getX()) + ", "
              + String.format("%.3f", wp.getY()) + ")");
        }
        System.out.println("==========================");
      } else {
        System.out.println(
            "[PathfindToTag] WARNING: Received invalid path (null or < 2 waypoints)");
      }
    }

    // If no path yet, wait (but NEVER drive directly - that would ignore obstacles!)
    if (!hasPath) {
      SmartDashboard.putString("Pathfinding/Status", "Waiting for path...");
      SmartDashboard.putNumber("Pathfinding/WaitTime", pathWaitTimer.get());

      // Keep publishing current pose even while waiting - VERY IMPORTANT for visibility!
      currentPosePublisher.set(currentPose);
      goalPosePublisher.set(targetPose);

      // Also publish to Field2d while waiting
      pathfindingField.setRobotPose(currentPose);
      pathfindingField.getObject("Goal").setPose(targetPose);
      pathfindingField.getObject("TargetWaypoint").setPose(targetPose); // No waypoint yet
      pathfindingField.getObject("Path").setPoses(); // Empty path

      // If waiting too long, log a warning but DON'T drive directly (that ignores obstacles)
      if (pathWaitTimer.hasElapsed(2.0) && shouldLogDebug) {
        System.out.println("[PathfindToTag] WARNING: Still waiting for path after "
            + String.format("%.1f", pathWaitTimer.get()) + "s");
      }
      return;
    }

    // Find current target waypoint using lookahead
    Translation2d targetWaypoint = findLookaheadPoint(currentPose);

    // Log state to SmartDashboard (works with AdvantageScope)
    SmartDashboard.putString("Pathfinding/Status", "Following path");
    SmartDashboard.putNumber("Pathfinding/CurrentWaypoint", currentWaypointIndex);
    SmartDashboard.putNumber("Pathfinding/TotalWaypoints", currentPath.size());

    // Calculate velocities using PID
    double xVelocity = clampVelocity(
        xController.calculate(currentPose.getX(), targetWaypoint.getX()),
        constraints.maxVelocityMps());

    double yVelocity = clampVelocity(
        yController.calculate(currentPose.getY(), targetWaypoint.getY()),
        constraints.maxVelocityMps());

    // Rotation towards goal pose
    double rotationVelocity = clampVelocity(
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()),
        constraints.maxAngularVelocityRadPerSec());

    // === DEBUG: Log control outputs ===
    if (shouldLogDebug) {
      System.out.println("\n===== PATH FOLLOWING DEBUG =====");
      System.out.println("[Follow] Current Pose: (" + String.format("%.3f", currentPose.getX())
          + ", " + String.format("%.3f", currentPose.getY()) + ", "
          + String.format("%.1f", currentPose.getRotation().getDegrees()) + "°)");
      System.out.println(
          "[Follow] Target Waypoint: (" + String.format("%.3f", targetWaypoint.getX()) + ", "
              + String.format("%.3f", targetWaypoint.getY()) + ")");
      System.out.println("[Follow] Goal Pose: (" + String.format("%.3f", targetPose.getX()) + ", "
          + String.format("%.3f", targetPose.getY()) + ", "
          + String.format("%.1f", targetPose.getRotation().getDegrees()) + "°)");
      System.out.println("[Follow] Position Error: X="
          + String.format("%.3f", targetWaypoint.getX() - currentPose.getX()) + ", Y="
          + String.format("%.3f", targetWaypoint.getY() - currentPose.getY()));
      System.out.println("[Follow] Velocity Commands (field-relative): vX="
          + String.format("%.3f", xVelocity) + ", vY=" + String.format("%.3f", yVelocity)
          + ", omega=" + String.format("%.3f", rotationVelocity));
      System.out.println("[Follow] Waypoint Index: " + currentWaypointIndex + "/"
          + (currentPath != null ? currentPath.size() : 0));
      System.out.println("================================\n");
    }

    // Apply to drivetrain as field-relative speeds
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
    ChassisSpeeds robotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentPose.getRotation());

    // === DEBUG: Log robot-relative speeds being sent ===
    if (shouldLogDebug) {
      System.out.println("[Follow] Robot-Relative Speeds: vX="
          + String.format("%.3f", robotSpeeds.vxMetersPerSecond) + ", vY="
          + String.format("%.3f", robotSpeeds.vyMetersPerSecond) + ", omega="
          + String.format("%.3f", robotSpeeds.omegaRadiansPerSecond));
    }

    // === NETWORKTABLES LOGGING FOR ADVANTAGESCOPE ===
    // Publish poses for 2D field visualization
    currentPosePublisher.set(currentPose);
    goalPosePublisher.set(targetPose);
    targetWaypointPublisher.set(new Pose2d(targetWaypoint, targetPose.getRotation()));

    // === FIELD2D VISUALIZATION (works with AdvantageScope/Shuffleboard) ===
    // Update robot pose on field
    pathfindingField.setRobotPose(currentPose);
    // Goal pose shown as a separate object
    pathfindingField.getObject("Goal").setPose(targetPose);
    // Target waypoint shown as another object
    pathfindingField
        .getObject("TargetWaypoint")
        .setPose(new Pose2d(targetWaypoint, targetPose.getRotation()));
    // Full path trajectory
    if (currentPath != null && !currentPath.isEmpty()) {
      Pose2d[] pathPoses = new Pose2d[currentPath.size()];
      for (int i = 0; i < currentPath.size(); i++) {
        pathPoses[i] = new Pose2d(currentPath.get(i), targetPose.getRotation());
      }
      pathfindingField.getObject("Path").setPoses(pathPoses);
    } else {
      pathfindingField.getObject("Path").setPoses();
    }

    // Publish velocity data to SmartDashboard
    SmartDashboard.putNumber("Pathfinding/VelX", xVelocity);
    SmartDashboard.putNumber("Pathfinding/VelY", yVelocity);
    SmartDashboard.putNumber("Pathfinding/VelOmega", rotationVelocity);
    SmartDashboard.putNumber("Pathfinding/ErrorX", targetWaypoint.getX() - currentPose.getX());
    SmartDashboard.putNumber("Pathfinding/ErrorY", targetWaypoint.getY() - currentPose.getY());
    SmartDashboard.putNumber("Pathfinding/CurrentWaypointIdx", currentWaypointIndex);
    SmartDashboard.putNumber(
        "Pathfinding/PathLength", currentPath != null ? currentPath.size() : 0);

    // Publish path for raw struct visualization too
    if (currentPath != null && !currentPath.isEmpty()) {
      Pose2d[] pathPoses = new Pose2d[currentPath.size()];
      for (int i = 0; i < currentPath.size(); i++) {
        pathPoses[i] = new Pose2d(currentPath.get(i), targetPose.getRotation());
      }
      pathPublisher.set(pathPoses);
    } else {
      // Publish empty array when no path to clear stale data
      pathPublisher.set(new Pose2d[0]);
    }

    drivetrain.setControl(
        new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotSpeeds));

    // Advance waypoint if close enough
    advanceWaypointIfNeeded(currentPose);

    // Update pathfinder with current pose for dynamic replanning
    Pathfinding.setStartPose(currentPose);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds()
        .withSpeeds(new ChassisSpeeds()));

    if (interrupted) {
      System.out.println("[PathfindToTag] Command interrupted");
    } else {
      System.out.println("[PathfindToTag] Reached goal: " + formatPose(targetPose));
    }
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = drivetrain.getState().Pose;

    // Check position tolerance
    double positionError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    boolean positionOk = positionError < PathfindingConstants.POSITION_TOLERANCE_METERS;

    // Check rotation tolerance
    double rotationError =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    boolean rotationOk = rotationError < PathfindingConstants.ROTATION_TOLERANCE_RADIANS;

    return positionOk && rotationOk;
  }

  // ==================== Path Following Helpers ====================

  private Translation2d findLookaheadPoint(Pose2d currentPose) {
    if (currentPath == null || currentPath.isEmpty()) {
      return targetPose.getTranslation();
    }

    Translation2d robotPos = currentPose.getTranslation();

    // Start from current waypoint index
    for (int i = currentWaypointIndex; i < currentPath.size(); i++) {
      Translation2d waypoint = currentPath.get(i);
      double distance = robotPos.getDistance(waypoint);

      if (distance >= LOOKAHEAD_DISTANCE) {
        // Interpolate to exact lookahead distance
        if (i > 0) {
          Translation2d prev = currentPath.get(i - 1);
          Translation2d direction = waypoint.minus(prev);
          double segmentLength = direction.getNorm();
          if (segmentLength > 0.01) {
            // Find point on segment at lookahead distance from robot
            Translation2d toRobot = robotPos.minus(prev);
            double projection =
                toRobot.getX() * direction.getX() + toRobot.getY() * direction.getY();
            projection /= segmentLength * segmentLength;
            projection = Math.max(0, Math.min(1, projection));
            Translation2d closestPoint = prev.plus(direction.times(projection));

            // Move lookahead distance along segment from closest point
            Translation2d segmentDir = direction.div(segmentLength);
            return closestPoint.plus(segmentDir.times(LOOKAHEAD_DISTANCE));
          }
        }
        return waypoint;
      }
    }

    // Return final goal
    return targetPose.getTranslation();
  }

  private void advanceWaypointIfNeeded(Pose2d currentPose) {
    if (currentPath == null || currentWaypointIndex >= currentPath.size() - 1) {
      return;
    }

    Translation2d waypoint = currentPath.get(currentWaypointIndex);
    double distance = currentPose.getTranslation().getDistance(waypoint);

    if (distance < WAYPOINT_TOLERANCE) {
      currentWaypointIndex++;
    }
  }

  private void driveTowardsPose(Pose2d currentPose, Pose2d goal) {
    double xVelocity = clampVelocity(
        xController.calculate(currentPose.getX(), goal.getX()), constraints.maxVelocityMps() * 0.3);

    double yVelocity = clampVelocity(
        yController.calculate(currentPose.getY(), goal.getY()), constraints.maxVelocityMps() * 0.3);

    double rotationVelocity = clampVelocity(
        rotationController.calculate(
            currentPose.getRotation().getRadians(), goal.getRotation().getRadians()),
        constraints.maxAngularVelocityRadPerSec() * 0.3);

    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
    ChassisSpeeds robotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentPose.getRotation());

    drivetrain.setControl(
        new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotSpeeds));
  }

  private double clampVelocity(double velocity, double max) {
    return Math.max(-max, Math.min(max, velocity));
  }

  private String formatPose(Pose2d pose) {
    return String.format(
        "(%.2f, %.2f, %.1f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }
}
