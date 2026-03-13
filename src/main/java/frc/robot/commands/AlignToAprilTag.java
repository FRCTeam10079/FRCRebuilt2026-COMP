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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.LimelightHelpers;
import frc.robot.statemachine.DrivetrainMode;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

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
  private final CommandSwerveDrivetrain m_drivetrain;
  private final RobotStateMachine m_stateMachine;

  // Timer for logging/timeout
  private final Timer m_timer = new Timer();

  // PID Controllers for position control
  private final PIDController m_pidX = new PIDController(
      DrivetrainConstants.ALIGN_PID_KP,
      DrivetrainConstants.ALIGN_PID_KI,
      DrivetrainConstants.ALIGN_PID_KD);
  private final PIDController m_pidY = new PIDController(
      DrivetrainConstants.ALIGN_PID_KP,
      DrivetrainConstants.ALIGN_PID_KI,
      DrivetrainConstants.ALIGN_PID_KD);
  private final PIDController m_pidRotate = new PIDController(
      DrivetrainConstants.ALIGN_ROTATION_KP, 0, DrivetrainConstants.ALIGN_ROTATION_KD);

  // Target pose to align to
  private Pose2d m_targetPose;

  // Offset configuration
  private final AlignPosition m_alignPosition;

  // Target AprilTag info
  private int m_targetTagID;
  private boolean m_tagDetected = false;

  // Timeout duration (seconds) - prevents indefinite alignment attempts
  private static final double ALIGN_TIMEOUT_SECONDS = 3.0;

  /**
   * Creates a new AlignToAprilTag command
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param vision The vision subsystem
   * @param alignPosition LEFT, RIGHT, or CENTER offset from the AprilTag
   */
  public AlignToAprilTag(CommandSwerveDrivetrain drivetrain, AlignPosition alignPosition) {
    m_drivetrain = drivetrain;
    m_stateMachine = RobotStateMachine.getInstance();
    m_alignPosition = alignPosition;

    // Enable continuous input for rotation (-PI to PI are same point)
    m_pidRotate.enableContinuousInput(-Math.PI, Math.PI);

    // This command requires the drivetrain
    addRequirements(m_drivetrain);

    DataLogManager.log("[AlignToAprilTag] Created for " + m_alignPosition + " position");
  }

  /** Convenience constructor for CENTER alignment */
  public AlignToAprilTag(CommandSwerveDrivetrain drivetrain) {
    this(drivetrain, AlignPosition.CENTER);
  }

  @Override
  public void initialize() {
    // Start timer
    m_timer.restart();

    // Set state machine to vision tracking mode
    m_stateMachine.setDrivetrainMode(DrivetrainMode.VISION_TRACKING);
    m_stateMachine.setAlignedToTarget(false);

    // Get current robot pose
    Pose2d robotPose = m_drivetrain.getState().Pose;
    if (robotPose == null) {
      DataLogManager.log("[AlignToAprilTag] ERROR: Robot pose is null!");
      m_tagDetected = false;
      return;
    }

    // Determine which tags to search based on alliance
    // Prevents aligning to opponent's tags across the field
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    int[] allianceTags = isRed ? AprilTagMaps.RED_SIDE_TAGS : AprilTagMaps.BLUE_SIDE_TAGS;

    // Find the closest AprilTag within our alliance's tags
    double minDistance = Double.MAX_VALUE;
    m_targetTagID = -1;

    for (int id : allianceTags) {
      double[] tagData = AprilTagMaps.aprilTagMap.get(id);
      if (tagData == null) continue;
      Pose2d tagPose = new Pose2d(
          tagData[0] * Constants.INCHES_TO_METERS,
          tagData[1] * Constants.INCHES_TO_METERS,
          new Rotation2d(Math.toRadians(tagData[3])));

      double distance = calculateDistance(robotPose, tagPose);
      if (distance < minDistance) {
        minDistance = distance;
        m_targetTagID = id;
      }
    }

    // Check if a tag was found
    if (m_targetTagID == -1) {
      DataLogManager.log("[AlignToAprilTag] ERROR: No AprilTag found in map!");
      m_tagDetected = false;
      return;
    }

    DataLogManager.log("[AlignToAprilTag] Closest tag from odometry: " + m_targetTagID);

    // Check if Limelight sees a valid tag - prefer it over odometry,
    // but only if it's on our alliance side
    int limelightTagID = 0;
    for (String name : VisionConstants.LIMELIGHT_NAMES) {
      int fid = (int) LimelightHelpers.getFiducialID(name);
      if (fid != 0) {
        limelightTagID = fid;
        break;
      }
    }
    if (limelightTagID != 0
        && AprilTagMaps.aprilTagMap.containsKey(limelightTagID)
        && Constants.contains(allianceTags, limelightTagID)) {
      m_targetTagID = limelightTagID;
      DataLogManager.log("[AlignToAprilTag] Using Limelight tag: " + m_targetTagID);
    } else if (limelightTagID == 0) {
      DataLogManager.log("[AlignToAprilTag] Limelight has no target, using odometry closest tag: "
          + m_targetTagID);
    } else {
      DataLogManager.log("[AlignToAprilTag] Limelight tag "
          + limelightTagID
          + " not valid for alliance, using odometry: "
          + m_targetTagID);
    }

    // Get tag data
    double[] tagData = AprilTagMaps.aprilTagMap.get(m_targetTagID);
    if (tagData == null) {
      DataLogManager.log("[AlignToAprilTag] ERROR: Tag data is null for ID: " + m_targetTagID);
      m_tagDetected = false;
      return;
    }

    // Convert tag position to Pose2d
    Pose2d aprilTagPose = new Pose2d(
        tagData[0] * Constants.INCHES_TO_METERS,
        tagData[1] * Constants.INCHES_TO_METERS,
        new Rotation2d(Math.toRadians(tagData[3])));

    m_tagDetected = true;

    double offsetX;
    double offsetY;

    // Calculate offset based on alignment position
    // Offsets are relative to the tag's coordinate frame
    switch (m_alignPosition) {
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
    m_targetPose = new Pose2d(
        aprilTagPose.getX() + rotatedOffsetX,
        aprilTagPose.getY() + rotatedOffsetY,
        new Rotation2d(targetRotation));

    // Set PID setpoints
    m_pidX.setSetpoint(m_targetPose.getX());
    m_pidY.setSetpoint(m_targetPose.getY());
    m_pidRotate.setSetpoint(m_targetPose.getRotation().getRadians());

    DataLogManager.log("[AlignToAprilTag] Target Pose: X="
        + m_targetPose.getX()
        + ", Y="
        + m_targetPose.getY()
        + ", Yaw="
        + Math.toDegrees(m_targetPose.getRotation().getRadians())
        + "°");

    // Log to SmartDashboard
    SmartDashboard.putNumber("AlignToAprilTag/TargetTagID", m_targetTagID);
    SmartDashboard.putNumber("AlignToAprilTag/TargetX", m_targetPose.getX());
    SmartDashboard.putNumber("AlignToAprilTag/TargetY", m_targetPose.getY());
    SmartDashboard.putNumber(
        "AlignToAprilTag/TargetYaw", m_targetPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    // If no tag detected, don't execute
    if (!m_tagDetected) {
      return;
    }

    // Get current robot pose
    Pose2d currentPose = m_drivetrain.getState().Pose;
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
    SmartDashboard.putNumber("AlignToAprilTag/ErrorX", m_pidX.getError());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorY", m_pidY.getError());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorYaw", Math.toDegrees(m_pidRotate.getError()));

    // Apply velocities to drivetrain
    m_drivetrain.setControl(new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withVelocityX(velocities[0])
        .withVelocityY(velocities[1])
        .withRotationalRate(velocities[2]));
  }

  /** Calculate velocities using PID control */
  private double[] calculatePIDVelocities(Pose2d currentPose) {
    // Calculate X velocity
    double velocityX = m_pidX.calculate(currentPose.getX());
    velocityX = MathUtil.clamp(
        velocityX, -DrivetrainConstants.ALIGN_SPEED_MPS, DrivetrainConstants.ALIGN_SPEED_MPS);

    // Calculate Y velocity
    double velocityY = m_pidY.calculate(currentPose.getY());
    velocityY = MathUtil.clamp(
        velocityY, -DrivetrainConstants.ALIGN_SPEED_MPS, DrivetrainConstants.ALIGN_SPEED_MPS);

    // Calculate rotation velocity
    double velocityYaw = m_pidRotate.calculate(currentPose.getRotation().getRadians());
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
    if (!m_tagDetected) {
      return true;
    }

    // Get current pose
    Pose2d currentPose = m_drivetrain.getState().Pose;
    if (currentPose == null || m_targetPose == null) {
      return true;
    }

    // Calculate position and yaw error
    double distance = m_targetPose.getTranslation().getDistance(currentPose.getTranslation());
    double yawError = Math.abs(MathUtil.angleModulus(
        m_targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));

    // Check if within tolerance
    boolean positionReached = distance <= DrivetrainConstants.POSITION_TOLERANCE_METERS;
    boolean yawReached = yawError <= DrivetrainConstants.YAW_TOLERANCE_RADIANS;

    // Timeout after ALIGN_TIMEOUT_SECONDS to prevent indefinite alignment attempts
    boolean timedOut = m_timer.hasElapsed(ALIGN_TIMEOUT_SECONDS);
    if (timedOut) {
      DataLogManager.log("[AlignToAprilTag] Timed out after " + ALIGN_TIMEOUT_SECONDS + "s"
          + " (distance=" + String.format("%.3f", distance)
          + "m, yawError=" + String.format("%.1f", Math.toDegrees(yawError)) + "°)");
    }

    SmartDashboard.putNumber("AlignToAprilTag/Distance", distance);
    SmartDashboard.putBoolean("AlignToAprilTag/PositionReached", positionReached);
    SmartDashboard.putBoolean("AlignToAprilTag/YawReached", yawReached);

    return (positionReached && yawReached) || timedOut;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    m_drivetrain.setControl(
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity));

    // Return to field-centric drive mode
    m_stateMachine.setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);

    // Set alignment status based on completion
    if (!interrupted && m_tagDetected) {
      m_stateMachine.setAlignedToTarget(true);
      DataLogManager.log(
          "[AlignToAprilTag] Completed successfully - aligned to tag " + m_targetTagID);
    } else {
      m_stateMachine.setAlignedToTarget(false);
      DataLogManager.log("[AlignToAprilTag] "
          + (interrupted ? "Interrupted" : "Failed")
          + " - alignment not confirmed");
    }

    // Log completion
    SmartDashboard.putBoolean("AlignToAprilTag/Completed", !interrupted);
    SmartDashboard.putNumber("AlignToAprilTag/Duration", m_timer.get());
  }
}
