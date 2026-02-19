// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

/**
 * Subsystem for dual Limelight vision cameras. Handles AprilTag detection and robot pose estimation
 * from both "limelightLeft" and "limelightRight".
 *
 * <p>Getter methods automatically return data from the camera that currently has the best target
 * (closest / largest area). For REBUILT 2026 season - used for hub alignment.
 */
public class LimelightSubsystem extends SubsystemBase {

  // NetworkTables for both Limelights
  private final NetworkTable leftTable;
  private final NetworkTable rightTable;

  // Basic targeting data entries - LEFT
  private final NetworkTableEntry leftTid, leftTx, leftTy, leftTa, leftTv;
  private final NetworkTableEntry leftBotPose, leftBotPoseFieldBlue, leftActivePipeline;

  // Basic targeting data entries - RIGHT
  private final NetworkTableEntry rightTid, rightTx, rightTy, rightTa, rightTv;
  private final NetworkTableEntry rightBotPose, rightBotPoseFieldBlue, rightActivePipeline;

  // Reference to drivetrain (used by other subsystems for alignment)
  private CommandSwerveDrivetrain drivetrain;

  /** The name of the camera currently providing the best target. */
  private String activeCameraName = VisionConstants.LIMELIGHT_LEFT_NAME;

  /** Creates a new LimelightSubsystem with dual cameras */
  public LimelightSubsystem() {
    // Get both Limelight NetworkTables
    leftTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_LEFT_NAME);
    rightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_RIGHT_NAME);

    // Initialize LEFT NetworkTable entries
    leftTid = leftTable.getEntry("tid");
    leftTx = leftTable.getEntry("tx");
    leftTy = leftTable.getEntry("ty");
    leftTa = leftTable.getEntry("ta");
    leftTv = leftTable.getEntry("tv");
    leftBotPose = leftTable.getEntry("botpose_targetspace");
    leftBotPoseFieldBlue = leftTable.getEntry("botpose_wpiblue");
    leftActivePipeline = leftTable.getEntry("getpipe");

    // Initialize RIGHT NetworkTable entries
    rightTid = rightTable.getEntry("tid");
    rightTx = rightTable.getEntry("tx");
    rightTy = rightTable.getEntry("ty");
    rightTa = rightTable.getEntry("ta");
    rightTv = rightTable.getEntry("tv");
    rightBotPose = rightTable.getEntry("botpose_targetspace");
    rightBotPoseFieldBlue = rightTable.getEntry("botpose_wpiblue");
    rightActivePipeline = rightTable.getEntry("getpipe");

    // Configure both Limelights
    for (String llName : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.setPipelineIndex(llName, VisionConstants.PIPELINE_APRILTAG);
      LimelightHelpers.setLEDMode_PipelineControl(llName);
      LimelightHelpers.setLEDMode_ForceOff(llName);
    }
  }

  /**
   * Set the drivetrain reference for vision-based odometry updates
   *
   * @param drivetrain The CommandSwerveDrivetrain instance
   */
  public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    // Determine which camera has the best target (largest area = closest).
    // This drives all getter methods so the rest of the code sees the "best" view.
    boolean leftValid = leftTv.getDouble(0) == 1.0 && ((int) leftTid.getDouble(0)) != 0;
    boolean rightValid = rightTv.getDouble(0) == 1.0 && ((int) rightTid.getDouble(0)) != 0;

    if (leftValid && rightValid) {
      // Both see a target — pick the one with the larger target area (closer)
      activeCameraName = leftTa.getDouble(0) >= rightTa.getDouble(0)
          ? VisionConstants.LIMELIGHT_LEFT_NAME
          : VisionConstants.LIMELIGHT_RIGHT_NAME;
    } else if (rightValid) {
      activeCameraName = VisionConstants.LIMELIGHT_RIGHT_NAME;
    } else {
      // Default to left (includes case where neither has a target)
      activeCameraName = VisionConstants.LIMELIGHT_LEFT_NAME;
    }

    // Update SmartDashboard with best-camera vision data
    SmartDashboard.putString("Limelight/ActiveCamera", activeCameraName);
    SmartDashboard.putNumber("Limelight/TX", getTx());
    SmartDashboard.putNumber("Limelight/TY", getTy());
    SmartDashboard.putNumber("Limelight/TA", getTa());
    SmartDashboard.putNumber("Limelight/TID", getTid());
    SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget());
    SmartDashboard.putNumber("Limelight/Yaw", getYaw());
    SmartDashboard.putBoolean("Limelight/LeftHasTarget", leftValid);
    SmartDashboard.putBoolean("Limelight/RightHasTarget", rightValid);

    // WATCH OUT! Vision-based odometry updates are now handled in
    // CommandSwerveDrivetrain.updateVision()
    // This prevents duplicate measurements and ensures proper MegaTag2 integration
    // with:
    // - SetRobotOrientation called before reading pose
    // - Dynamic standard deviations based on tag distance/count
    // - Angular velocity rejection for MegaTag2 accuracy
    // - Field boundary validation
  }

  // ==================== ACTIVE CAMERA HELPERS ====================

  /** @return true if the active camera is the left Limelight */
  private boolean isLeftActive() {
    return VisionConstants.LIMELIGHT_LEFT_NAME.equals(activeCameraName);
  }

  /** @return The name of the camera currently providing the best target */
  public String getActiveCameraName() {
    return activeCameraName;
  }

  // ==================== GETTER METHODS (auto-select best camera)
  // ====================

  /** @return The robot pose relative to the target [tx, ty, tz, pitch, yaw, roll] */
  public double[] getBotPose() {
    return (isLeftActive() ? leftBotPose : rightBotPose).getDoubleArray(new double[6]);
  }

  /**
   * @return The robot pose on field from Blue alliance origin [X, Y, Z, Roll, Pitch, Yaw, Latency,
   *     Tag Count, Tag Span, Avg Tag Distance, Avg Tag Area]
   */
  public double[] getBotPoseFieldBlue() {
    return (isLeftActive() ? leftBotPoseFieldBlue : rightBotPoseFieldBlue)
        .getDoubleArray(new double[11]);
  }

  /** @return The yaw of the robot relative to the target (degrees) */
  public double getYaw() {
    return getBotPose()[4];
  }

  /** @return Horizontal offset from crosshair to target (degrees) */
  public double getTx() {
    return (isLeftActive() ? leftTx : rightTx).getDouble(0);
  }

  /** @return Vertical offset from crosshair to target (degrees) */
  public double getTy() {
    return (isLeftActive() ? leftTy : rightTy).getDouble(0);
  }

  /** @return Target area as percentage of image (0-100) */
  public double getTa() {
    return (isLeftActive() ? leftTa : rightTa).getDouble(0);
  }

  /** @return The AprilTag ID being tracked (0 if none) */
  public int getTid() {
    return (int) (isLeftActive() ? leftTid : rightTid).getDouble(0);
  }

  /** @return True if a valid target is detected on either camera */
  public boolean hasTarget() {
    boolean leftValid = leftTv.getDouble(0) == 1.0 && ((int) leftTid.getDouble(0)) != 0;
    boolean rightValid = rightTv.getDouble(0) == 1.0 && ((int) rightTid.getDouble(0)) != 0;
    return leftValid || rightValid;
  }

  /** @return True if an AprilTag is detected (alias for hasTarget) */
  public boolean isTagDetected() {
    return hasTarget();
  }

  /** @return Number of tags visible in current frame (from active camera) */
  public double getTagCount() {
    return getBotPoseFieldBlue()[7];
  }

  /** @return Robot pose as Pose2d (X, Y, Yaw on field) from active camera */
  public Pose2d getPose() {
    double[] poseData = getBotPoseFieldBlue();
    Rotation2d rotation = new Rotation2d(Math.toRadians(poseData[5]));
    return new Pose2d(poseData[0], poseData[1], rotation);
  }

  /** @return Current active pipeline index of the active camera */
  public int getActivePipeline() {
    return (int) (isLeftActive() ? leftActivePipeline : rightActivePipeline).getDouble(0);
  }

  /**
   * Set the pipeline on both Limelights
   *
   * @param pipelineIndex Pipeline index (0-9)
   */
  public void setPipeline(int pipelineIndex) {
    for (String llName : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.setPipelineIndex(llName, pipelineIndex);
    }
  }

  /** Turn LEDs on (both cameras) */
  public void setLEDsOn() {
    for (String llName : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.setLEDMode_ForceOn(llName);
    }
  }

  /** Turn LEDs off (both cameras) */
  public void setLEDsOff() {
    for (String llName : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.setLEDMode_ForceOff(llName);
    }
  }

  /** Set LEDs to blink (both cameras) */
  public void setLEDsBlink() {
    for (String llName : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.setLEDMode_ForceBlink(llName);
    }
  }
}
