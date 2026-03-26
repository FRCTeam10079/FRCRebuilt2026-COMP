// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  private static final double FIELD_MARGIN = VisionConstants.FIELD_BORDER_MARGIN;
  private static final double FIELD_LENGTH = VisionConstants.FIELD_LENGTH_METERS;
  private static final double FIELD_WIDTH = VisionConstants.FIELD_WIDTH_METERS;

  private static final int PIPELINE_APRILTAG = 0;

  private final CommandSwerveDrivetrain drivetrain;

  private int totalAccepted = 0;
  private int totalRejected = 0;
  private int headingCorrections = 0;
  private boolean hasBootstrappedHeading = false;

  private boolean m_ignoreVision = false; // new code: flag to ignore vision

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    String[] names = VisionConstants.LIMELIGHT_NAMES;
    for (String name : names) {
      LimelightHelpers.setPipelineIndex(name, PIPELINE_APRILTAG);
      LimelightHelpers.setLEDMode_PipelineControl(name);
      LimelightHelpers.setLEDMode_ForceOff(name);
      LimelightHelpers.SetIMUMode(name, 0);
    }
  }

  @Override
  public void periodic() {
    boolean isDisabled = edu.wpi.first.wpilibj.DriverStation.isDisabled();
    Logger.recordOutput("Vision/DisabledSkipped", isDisabled);
    // Vision fusion must stay enabled in auto so PathPlanner pose correction can
    // run.
    if (isDisabled || m_ignoreVision) {
      return;
    }

    String[] names = VisionConstants.LIMELIGHT_NAMES;
    Pose2d odoPose = drivetrain.getState().Pose;
    ChassisSpeeds speeds = drivetrain.getState().Speeds;

    for (String name : names) {
      processCamera(name, odoPose, speeds);
    }

    // Avoid System.out in fast loops; it adds roboRIO CPU overhead and AdvantageKit
    // already logs
    // this data.

    Logger.recordOutput("Vision/TotalAccepted", totalAccepted);
    Logger.recordOutput("Vision/TotalRejected", totalRejected);
    Logger.recordOutput("Vision/HeadingCorrections", headingCorrections);
  }

  private void processCamera(String cameraName, Pose2d odoPose, ChassisSpeeds speeds) {
    String logPrefix = "Vision/" + cameraName + "/";

    // MT1 does not use SetRobotOrientation cuz it computes heading from vision
    // alone.
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (mt1 == null || mt1.timestampSeconds == 0 || mt1.tagCount == 0) {
      Logger.recordOutput(logPrefix + "Status", "NO_DATA");
      return;
    }

    Pose2d pose = mt1.pose;
    double avgTagDist = mt1.avgTagDist;

    // ---- Rejection checks ----

    // Reject if robot is spinning too fast for reliable vision
    double omegaDegPerSec = Math.toDegrees(Math.abs(speeds.omegaRadiansPerSecond));
    if (omegaDegPerSec > VisionConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
      Logger.recordOutput(logPrefix + "Status", "REJECTED_ANGULAR_VELOCITY");
      totalRejected++;
      return;
    }

    // Pose outside field bounds (with margin) = clearly wrong
    if (pose.getX() < -FIELD_MARGIN
        || pose.getX() > FIELD_LENGTH + FIELD_MARGIN
        || pose.getY() < -FIELD_MARGIN
        || pose.getY() > FIELD_WIDTH + FIELD_MARGIN) {
      Logger.recordOutput(logPrefix + "Status", "REJECTED_OUT_OF_FIELD");
      totalRejected++;
      return;
    }

    // Hard distance gate on single tags
    if (mt1.tagCount == 1 && avgTagDist > 2.5) {
      Logger.recordOutput(logPrefix + "Status", "REJECTED_DISTANCE");
      totalRejected++;
      return;
    }

    // Innovation gate: reject if vision pose is too far from current odometry pose
    double innovationMeters = pose.getTranslation().getDistance(odoPose.getTranslation());
    double innovationThreshold = mt1.tagCount >= 2 ? 1.0 : 0.75;
    innovationThreshold += avgTagDist * 0.15; // widen gate proportionally with distance
    
    if (innovationMeters > innovationThreshold) {
      Logger.recordOutput(logPrefix + "Status", "REJECTED_INNOVATION");
      totalRejected++;
      return;
    }

    if (mt1.tagCount == 1) {
      // Bounds safety for raw fiducials and measurement rejection are separate
      // concerns: malformed
      // single-tag frames must still be rejected, not allowed to bypass single-tag
      // gates.
      if (mt1.rawFiducials == null
          || mt1.rawFiducials.length < 1
          || mt1.rawFiducials[0].ambiguity > VisionConstants.MT1_AMBIGUITY_THRESHOLD) {
        Logger.recordOutput(logPrefix + "Status", "REJECTED_AMBIGUITY");
        totalRejected++;
        return;
      }
      if (avgTagDist > VisionConstants.MAX_MT1_DISTANCE_METERS) {
        Logger.recordOutput(logPrefix + "Status", "REJECTED_DISTANCE");
        totalRejected++;
        return;
      }
    }

    // ---- One-shot heading bootstrap ----
    // On first reliable multi-tag result, correct gyro if heading is way off.
    // This handles the Pigeon2 booting to 0° when the actual heading is ~180°.
    if (!hasBootstrappedHeading && mt1.tagCount >= 2) {
      double divergenceDeg = Math.abs(MathUtil.inputModulus(
          pose.getRotation().getDegrees() - odoPose.getRotation().getDegrees(), -180, 180));
      if (divergenceDeg > VisionConstants.HEADING_BOOTSTRAP_THRESHOLD_DEG) {
        Pose2d correctedPose = new Pose2d(odoPose.getTranslation(), pose.getRotation());
        drivetrain.resetPose(correctedPose);
        headingCorrections++;
        DataLogManager.log("[Vision] One-shot heading bootstrap: "
            + String.format("%.1f", odoPose.getRotation().getDegrees())
            + "° -> "
            + String.format("%.1f", pose.getRotation().getDegrees())
            + "°");
        Logger.recordOutput(logPrefix + "HeadingBootstrap", true);
      }
      hasBootstrappedHeading = true;
    }

    // ---- Standard deviation model ----
    // dist^1.2 scaling, inversely proportional to tagCount^2.
    double xyStdev = VisionConstants.XY_STDDEV_COEFFICIENT
        * Math.pow(avgTagDist, VisionConstants.XY_STDDEV_EXPONENT)
        / (mt1.tagCount * mt1.tagCount);

    // Never trust MT1 heading - coplanar tag ambiguity can flip it 180 degrees.
    // Gyro (Pigeon2) is the sole heading authority.
    // This matches Limelight's own official MT1 example:
    // VecBuilder.fill(.5,.5,9999999)
    double thetaStdDev = Double.POSITIVE_INFINITY;

    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdev, xyStdev, thetaStdDev);

    drivetrain.addVisionMeasurement(pose, mt1.timestampSeconds, stdDevs);

    totalAccepted++;
    Logger.recordOutput(logPrefix + "Status", "ACCEPTED");
    Logger.recordOutput(logPrefix + "Pose", pose);
    Logger.recordOutput(logPrefix + "TagCount", mt1.tagCount);
    Logger.recordOutput(logPrefix + "AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "XYStdDev", xyStdev);
    Logger.recordOutput(logPrefix + "ThetaStdDev", thetaStdDev);
  }

  public void updateWhileDisabled() {
    String[] names = VisionConstants.LIMELIGHT_NAMES;
    Pose2d currentPose = drivetrain.getState().Pose;
    double currentHeadingDeg = currentPose.getRotation().getDegrees();

    for (String name : names) {
      LimelightHelpers.SetRobotOrientation(name, currentHeadingDeg, 0, 0, 0, 0, 0);
      // Mode 1 while disabled: seeds LL4's internal IMU with external gyro.
      LimelightHelpers.SetIMUMode(name, 1);

      if (VisionConstants.USE_MT1_HEADING_CORRECTION_WHILE_DISABLED) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (!hasBootstrappedHeading && mt1.tagCount >= 2 && mt1.timestampSeconds != 0) {
          double mt1HeadingDeg = mt1.pose.getRotation().getDegrees();

          Logger.recordOutput("Vision/" + name + "/Disabled/MT1HeadingDeg", mt1HeadingDeg);

          // This path bypasses processCamera(), so gate protections must be applied
          // manually; use
          // distance as the primary protection because odometry is static while disabled.
          if (mt1.avgTagDist > VisionConstants.MAX_MT1_DISTANCE_METERS) {
            Logger.recordOutput("Vision/" + name + "/Disabled/HeadingCorrected", false);
            Logger.recordOutput("Vision/" + name + "/Disabled/Status", "REJECTED_DISTANCE");
            totalRejected++;
            continue;
          }
          
          drivetrain.resetPose(mt1.pose);

          hasBootstrappedHeading = true;
          headingCorrections++;
          DataLogManager.log("[Vision] Auto-seeded full pose from MT1 multi-tag: X=" 
              + String.format("%.2f", mt1.pose.getX()) + " Y=" + String.format("%.2f", mt1.pose.getY()) 
              + " Theta=" + String.format("%.1f", mt1.pose.getRotation().getDegrees())
              + "° (camera: " + name + ")");
          Logger.recordOutput("Vision/" + name + "/Disabled/HeadingCorrected", true);

          break;
        }
      }
    }
  }

  // new code: Allow pausing vision integration during manual overrides
  public void setIgnoreVision(boolean ignore) {
    m_ignoreVision = ignore;
  }
}
