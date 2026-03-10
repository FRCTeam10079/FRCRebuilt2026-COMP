// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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
    String[] names = VisionConstants.LIMELIGHT_NAMES;
    Pose2d odoPose = drivetrain.getState().Pose;

    for (String name : names) {
      processCamera(name, odoPose);
    }

    Logger.recordOutput("Vision/TotalAccepted", totalAccepted);
    Logger.recordOutput("Vision/TotalRejected", totalRejected);
    Logger.recordOutput("Vision/HeadingCorrections", headingCorrections);
  }

  private void processCamera(String cameraName, Pose2d odoPose) {
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

    // Pose outside field bounds (with margin) = clearly wrong
    if (pose.getX() < -FIELD_MARGIN
        || pose.getX() > FIELD_LENGTH + FIELD_MARGIN
        || pose.getY() < -FIELD_MARGIN
        || pose.getY() > FIELD_WIDTH + FIELD_MARGIN) {
      Logger.recordOutput(logPrefix + "Status", "REJECTED_OUT_OF_FIELD");
      totalRejected++;
      return;
    }

    // Single-tag: reject high-ambiguity
    if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
      if (mt1.rawFiducials[0].ambiguity > VisionConstants.MT1_AMBIGUITY_THRESHOLD) {
        Logger.recordOutput(logPrefix + "Status", "REJECTED_AMBIGUITY");
        totalRejected++;
        return;
      }
    }

    // ---- Standard deviation model ----
    // dist^1.2 scaling, inversely proportional to tagCount^2.
    double xyStdev = VisionConstants.XY_STDDEV_COEFFICIENT
        * Math.pow(avgTagDist, VisionConstants.XY_STDDEV_EXPONENT)
        / (mt1.tagCount * mt1.tagCount);

    // Heading: trust multi-tag MT1 heading to correct gyro drift.
    // Single-tag heading is ambiguity-prone so don't trust.
    double thetaStdDev;
    if (mt1.tagCount >= 2) {
      thetaStdDev = VisionConstants.THETA_STDDEV_COEFFICIENT
          * Math.pow(avgTagDist, VisionConstants.XY_STDDEV_EXPONENT)
          / (mt1.tagCount * mt1.tagCount);
    } else {
      thetaStdDev = Double.POSITIVE_INFINITY;
    }

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
      LimelightHelpers.SetIMUMode(name, 0);

      if (VisionConstants.USE_MT1_HEADING_CORRECTION_WHILE_DISABLED) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (mt1.tagCount >= 2 && mt1.timestampSeconds != 0) {
          double mt1HeadingDeg = mt1.pose.getRotation().getDegrees();
          double divergenceDeg =
              Math.abs(MathUtil.inputModulus(mt1HeadingDeg - currentHeadingDeg, -180, 180));

          Logger.recordOutput("Vision/" + name + "/Disabled/MT1HeadingDeg", mt1HeadingDeg);
          Logger.recordOutput("Vision/" + name + "/Disabled/HeadingDivergenceDeg", divergenceDeg);

          if (divergenceDeg > VisionConstants.MT1_HEADING_CORRECTION_THRESHOLD_DEG) {
            Pose2d correctedPose = new Pose2d(currentPose.getTranslation(), mt1.pose.getRotation());
            drivetrain.resetPose(correctedPose);

            headingCorrections++;
            DataLogManager.log("[Vision] Auto-corrected heading from MT1 multi-tag: "
                + String.format("%.1f", currentHeadingDeg)
                + "° -> "
                + String.format("%.1f", mt1HeadingDeg)
                + "° (divergence: "
                + String.format("%.1f", divergenceDeg)
                + "°, camera: "
                + name
                + ")");
            Logger.recordOutput("Vision/" + name + "/Disabled/HeadingCorrected", true);

            break;
          }
        }
      }
    }
  }
}
