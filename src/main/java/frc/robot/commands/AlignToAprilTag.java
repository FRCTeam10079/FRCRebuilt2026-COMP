// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.RobotStateMachine;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to align the robot to an AprilTag using vision
 *
 * <p>This command: 1. Finds the closest AprilTag (or uses Limelight-detected tag) 2. Calculates
 * target position with optional offset (LEFT/RIGHT/CENTER) 3. Uses PID control to drive the robot
 * to the target pose 4. Rotates to face opposite the tag (facing the tag)
 *
 * <p>For REBUILT 2026 - Generic AprilTag alignment for any field element
 */
public class AlignToAprilTag extends Command {

  // Subsystems
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;
  private final RobotStateMachine stateMachine;

  // Timer for logging/timeout
  private final Timer timer = new Timer();

  // PID Controllers for position control
  private final PIDController pidX;
  private final PIDController pidY;
  private final PIDController pidRotate;

  // Swerve drive request - field centric with velocity control
  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  // Stop request
  private final SwerveRequest stop;

  // Target pose to align to
  private Pose2d targetPose;

  // Configuration from Constants
  private final double speed;
  private final double rotationSpeed;
  private final double positionTolerance;
  private final double yawTolerance;

  // Offset configuration
  private final AlignPosition alignPosition;
  private double offsetX = 0;
  private double offsetY = 0;

  // Target AprilTag info
  private int targetTagID;
  private boolean tagDetected = false;

  // Store initial tag to prevent switching during alignment
  private Integer storedTagID = null;

  /**
   * Creates a new AlignToAprilTag command
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param limelight The limelight vision subsystem
   * @param alignPosition LEFT, RIGHT, or CENTER offset from the AprilTag
   */
  public AlignToAprilTag(
      CommandSwerveDrivetrain drivetrain,
      LimelightSubsystem limelight,
      AlignPosition alignPosition) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.stateMachine = RobotStateMachine.getInstance();
    this.alignPosition = alignPosition;

    // Get constants
    this.speed = DrivetrainConstants.ALIGN_SPEED_MPS;
    this.rotationSpeed = DrivetrainConstants.ALIGN_ROTATION_SPEED;
    this.positionTolerance = DrivetrainConstants.POSITION_TOLERANCE_METERS;
    this.yawTolerance = DrivetrainConstants.YAW_TOLERANCE_RADIANS;

    // Initialize PID controllers with values from Constants
    pidX = new PIDController(
        DrivetrainConstants.ALIGN_PID_KP,
        DrivetrainConstants.ALIGN_PID_KI,
        DrivetrainConstants.ALIGN_PID_KD);
    pidY = new PIDController(
        DrivetrainConstants.ALIGN_PID_KP,
        DrivetrainConstants.ALIGN_PID_KI,
        DrivetrainConstants.ALIGN_PID_KD);
    pidRotate = new PIDController(
        DrivetrainConstants.ALIGN_ROTATION_KP, 0, DrivetrainConstants.ALIGN_ROTATION_KD);

    // Enable continuous input for rotation (-PI to PI are same point)
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);

    // Create stop request
    stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

    // This command requires the drivetrain
    addRequirements(drivetrain);

    System.out.println("[AlignToAprilTag] Created for " + alignPosition + " position");
  }

  /** Convenience constructor for CENTER alignment */
  public AlignToAprilTag(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
    this(drivetrain, limelight, AlignPosition.CENTER);
  }

  @Override
  public void initialize() {
    // Start timer
    timer.restart();

    // Set state machine to vision tracking mode
    stateMachine.setDrivetrainMode(RobotStateMachine.DrivetrainMode.VISION_TRACKING);
    stateMachine.setAlignedToTarget(false);

    // Reset stored tag
    storedTagID = null;

    // Get current robot pose
    Pose2d robotPose = drivetrain.getState().Pose;
    if (robotPose == null) {
      System.out.println("[AlignToAprilTag] ERROR: Robot pose is null!");
      tagDetected = false;
      return;
    }

    // Find the closest AprilTag using odometry
    double minDistance = Double.MAX_VALUE;
    targetTagID = -1;

    for (int id : AprilTagMaps.aprilTagMap.keySet()) {
      double[] tagData = AprilTagMaps.aprilTagMap.get(id);
      Pose2d tagPose = new Pose2d(
          tagData[0] * Constants.INCHES_TO_METERS,
          tagData[1] * Constants.INCHES_TO_METERS,
          new Rotation2d(Math.toRadians(tagData[3])));

      double distance = calculateDistance(robotPose, tagPose);
      if (distance < minDistance) {
        minDistance = distance;
        targetTagID = id;
      }
    }

    // Check if a tag was found
    if (targetTagID == -1) {
      System.out.println("[AlignToAprilTag] ERROR: No AprilTag found in map!");
      tagDetected = false;
      return;
    }

    System.out.println("[AlignToAprilTag] Closest tag from odometry: " + targetTagID);

    // Check if Limelight sees a valid tag - prefer it over odometry
    int limelightTagID = limelight.getTid();
    if (limelightTagID != 0 && AprilTagMaps.aprilTagMap.containsKey(limelightTagID)) {
      targetTagID = limelightTagID;
      System.out.println("[AlignToAprilTag] Using Limelight tag: " + targetTagID);
    } else if (limelightTagID == 0) {
      System.out.println(
          "[AlignToAprilTag] Limelight has no target, using odometry closest tag: " + targetTagID);
    } else {
      System.out.println("[AlignToAprilTag] Limelight tag "
          + limelightTagID
          + " not in map, using odometry: "
          + targetTagID);
    }

    // Store the tag ID to prevent switching mid-alignment
    storedTagID = targetTagID;

    // Get tag data
    double[] tagData = AprilTagMaps.aprilTagMap.get(targetTagID);
    if (tagData == null) {
      System.out.println("[AlignToAprilTag] ERROR: Tag data is null for ID: " + targetTagID);
      tagDetected = false;
      return;
    }

    // Convert tag position to Pose2d
    Pose2d aprilTagPose = new Pose2d(
        tagData[0] * Constants.INCHES_TO_METERS,
        tagData[1] * Constants.INCHES_TO_METERS,
        new Rotation2d(Math.toRadians(tagData[3])));

    tagDetected = true;

    // Calculate offset based on alignment position
    // Offsets are relative to the tag's coordinate frame
    switch (alignPosition) {
      case LEFT:
        offsetX = DrivetrainConstants.ALIGN_OFFSET_X_LEFT;
        offsetY = DrivetrainConstants.ALIGN_OFFSET_Y_LEFT;
        break;
      case RIGHT:
        offsetX = DrivetrainConstants.ALIGN_OFFSET_X_RIGHT;
        offsetY = DrivetrainConstants.ALIGN_OFFSET_Y_RIGHT;
        break;
      case CENTER:
      default:
        offsetX = DrivetrainConstants.ALIGN_OFFSET_X_CENTER;
        offsetY = DrivetrainConstants.ALIGN_OFFSET_Y_CENTER;
        break;
    }

    // Target rotation is opposite the tag (facing the tag)
    double targetRotation = aprilTagPose.getRotation().getRadians() - Math.PI;
    targetRotation = MathUtil.angleModulus(targetRotation);

    // Rotate the offset from tag-relative to field-relative
    double rotatedOffsetX =
        (offsetX * Math.cos(targetRotation)) - (offsetY * Math.sin(targetRotation));
    double rotatedOffsetY =
        (offsetX * Math.sin(targetRotation)) + (offsetY * Math.cos(targetRotation));

    // Calculate final target pose
    targetPose = new Pose2d(
        aprilTagPose.getX() + rotatedOffsetX,
        aprilTagPose.getY() + rotatedOffsetY,
        new Rotation2d(targetRotation));

    // Set PID setpoints
    pidX.setSetpoint(targetPose.getX());
    pidY.setSetpoint(targetPose.getY());
    pidRotate.setSetpoint(targetPose.getRotation().getRadians());

    System.out.println("[AlignToAprilTag] Target Pose: X="
        + targetPose.getX()
        + ", Y="
        + targetPose.getY()
        + ", Yaw="
        + Math.toDegrees(targetPose.getRotation().getRadians())
        + "°");

    // Log to SmartDashboard
    SmartDashboard.putNumber("AlignToAprilTag/TargetTagID", targetTagID);
    SmartDashboard.putNumber("AlignToAprilTag/TargetX", targetPose.getX());
    SmartDashboard.putNumber("AlignToAprilTag/TargetY", targetPose.getY());
    SmartDashboard.putNumber(
        "AlignToAprilTag/TargetYaw", targetPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    // If no tag detected, don't execute
    if (!tagDetected) {
      return;
    }

    // Get current robot pose
    Pose2d currentPose = drivetrain.getState().Pose;
    if (currentPose == null) {
      return;
    }

    // Calculate velocities using PID
    double[] velocities = calculatePIDVelocities(currentPose);

    // Log data
    SmartDashboard.putNumber("AlignToAprilTag/CurrentX", currentPose.getX());
    SmartDashboard.putNumber("AlignToAprilTag/CurrentY", currentPose.getY());
    SmartDashboard.putNumber(
        "AlignToAprilTag/CurrentYaw", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorX", pidX.getError());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorY", pidY.getError());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorYaw", Math.toDegrees(pidRotate.getError()));

    // Check if we need to flip direction based on alliance side
    // Red side tags (6-11) may need direction adjustment
    boolean isRedSide = Constants.contains(new int[] {6, 7, 8, 9, 10, 11}, targetTagID);

    // Apply velocities to drivetrain
    drivetrain.setControl(driveRequest
        .withVelocityX(velocities[0])
        .withVelocityY(velocities[1])
        .withRotationalRate(velocities[2]));
  }

  /** Calculate velocities using PID control */
  private double[] calculatePIDVelocities(Pose2d currentPose) {
    // Calculate X velocity
    double velocityX = pidX.calculate(currentPose.getX());
    velocityX = MathUtil.clamp(velocityX, -speed, speed);

    // Calculate Y velocity
    double velocityY = pidY.calculate(currentPose.getY());
    velocityY = MathUtil.clamp(velocityY, -speed, speed);

    // Calculate rotation velocity
    double velocityYaw = pidRotate.calculate(currentPose.getRotation().getRadians());
    velocityYaw = MathUtil.clamp(velocityYaw, -2.0, 2.0);

    return new double[] {velocityX, velocityY, velocityYaw};
  }

  /** Calculate distance between two poses */
  private double calculateDistance(Pose2d pose1, Pose2d pose2) {
    double dx = pose1.getX() - pose2.getX();
    double dy = pose1.getY() - pose2.getY();
    return Math.sqrt(dx * dx + dy * dy);
  }

  @Override
  public boolean isFinished() {
    // End if no tag detected
    if (!tagDetected) {
      return true;
    }

    // Get current pose
    Pose2d currentPose = drivetrain.getState().Pose;
    if (currentPose == null || targetPose == null) {
      return true;
    }

    // Calculate position and yaw error
    double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
    double yawError = Math.abs(MathUtil.angleModulus(
        targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));

    // Check if within tolerance
    boolean positionReached = distance <= positionTolerance;
    boolean yawReached = yawError <= yawTolerance;

    SmartDashboard.putNumber("AlignToAprilTag/Distance", distance);
    SmartDashboard.putBoolean("AlignToAprilTag/PositionReached", positionReached);
    SmartDashboard.putBoolean("AlignToAprilTag/YawReached", yawReached);

    return positionReached && yawReached;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    drivetrain.setControl(stop);

    // Return to field-centric drive mode
    stateMachine.setDrivetrainMode(RobotStateMachine.DrivetrainMode.FIELD_CENTRIC);

    // Set alignment status based on completion
    if (!interrupted && tagDetected) {
      stateMachine.setAlignedToTarget(true);
      System.out.println(
          "[AlignToAprilTag] Completed successfully - aligned to tag " + targetTagID);
    } else {
      stateMachine.setAlignedToTarget(false);
      System.out.println("[AlignToAprilTag] "
          + (interrupted ? "Interrupted" : "Failed")
          + " - alignment not confirmed");
    }

    // Log completion
    SmartDashboard.putBoolean("AlignToAprilTag/Completed", !interrupted);
    SmartDashboard.putNumber("AlignToAprilTag/Duration", timer.get());
  }
}
