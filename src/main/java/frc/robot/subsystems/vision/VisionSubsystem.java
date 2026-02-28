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

  private int totalMt2Accepted = 0;
  private int totalMt1Accepted = 0;
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
    double fusedHeadingDeg = odoPose.getRotation().getDegrees();
    double angularVelocityDegPerSec =
        drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();

    for (String name : names) {
      processCamera(name, odoPose, fusedHeadingDeg, angularVelocityDegPerSec);
    }

    Logger.recordOutput("Vision/TotalMT2Accepted", totalMt2Accepted);
    Logger.recordOutput("Vision/TotalMT1Accepted", totalMt1Accepted);
    Logger.recordOutput("Vision/TotalRejected", totalRejected);
    Logger.recordOutput("Vision/HeadingCorrections", headingCorrections);
    Logger.recordOutput("Vision/FusedHeadingDeg", fusedHeadingDeg);
  }

  private void processCamera(
      String cameraName, Pose2d odoPose, double fusedHeadingDeg, double angularVelocityDegPerSec) {

    String logPrefix = "Vision/" + cameraName + "/";

    // SAFETY REMOVED: Accept even while rotating fast
    // if (Math.abs(angularVelocityDegPerSec) > VisionConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
    //   Logger.recordOutput(logPrefix + "Status", "ANGULAR_VEL_REJECT");
    //   totalRejected++;
    //   return;
    // }

    LimelightHelpers.SetRobotOrientation(
        cameraName, fusedHeadingDeg, angularVelocityDegPerSec, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    boolean mt2Accepted = tryAcceptMT2(mt2, odoPose, logPrefix);

    // Only fetch MT1 if MT2 was rejected (lazy-fetch saves bandwidth)
    if (!mt2Accepted) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
      tryAcceptMT1(mt1, odoPose, logPrefix);
    }
  }

  private boolean tryAcceptMT2(
      LimelightHelpers.PoseEstimate mt2, Pose2d odoPose, String logPrefix) {

    if (mt2 == null || mt2.timestampSeconds == 0 || mt2.tagCount == 0) {
      Logger.recordOutput(logPrefix + "MT2/Status", "NO_DATA");
      return false;
    }

    Pose2d pose = mt2.pose;

    // SAFETY REMOVED: Accept all poses unconditionally
    // if (pose.equals(new Pose2d())) {
    //   Logger.recordOutput(logPrefix + "MT2/Status", "EMPTY_POSE");
    //   totalRejected++;
    //   return false;
    // }

    // double x = pose.getX();
    // double y = pose.getY();
    // if (x < -FIELD_MARGIN
    //     || x > FIELD_LENGTH + FIELD_MARGIN
    //     || y < -FIELD_MARGIN
    //     || y > FIELD_WIDTH + FIELD_MARGIN) {
    //   Logger.recordOutput(logPrefix + "MT2/Status", "OUT_OF_BOUNDS");
    //   totalRejected++;
    //   return false;
    // }

    // double headingDivergenceDeg =
    //     Math.abs(odoPose.getRotation().minus(pose.getRotation()).getDegrees());
    // Logger.recordOutput(logPrefix + "MT2/HeadingDivergenceDeg", headingDivergenceDeg);

    // if (headingDivergenceDeg > VisionConstants.HEADING_DIVERGENCE_THRESHOLD_DEG) {
    //   Logger.recordOutput(logPrefix + "MT2/Status", "HEADING_DIVERGE");
    //   totalRejected++;
    //   return false;
    // }

    double avgTagDist = mt2.avgTagDist;

    // if (avgTagDist > VisionConstants.MAX_TAG_DISTANCE_METERS) {
    //   Logger.recordOutput(logPrefix + "MT2/Status", "DISTANCE_REJECT");
    //   totalRejected++;
    //   return false;
    // }

    // Pose-difference sanity check removed
    // double poseDifference = odoPose.getTranslation().getDistance(pose.getTranslation());
    // Logger.recordOutput(logPrefix + "MT2/PoseDifference", poseDifference);

    // if (poseDifference > VisionConstants.MAX_POSE_DIFFERENCE_METERS) {
    //   Logger.recordOutput(logPrefix + "MT2/Status", "POSE_DIFF_REJECT");
    //   totalRejected++;
    //   return false;
    // }

    // Tag area gating removed
    // if (mt2.tagCount == 1 && mt2.avgTagArea < VisionConstants.MIN_TAG_AREA_SINGLE_TAG) {
    //   Logger.recordOutput(logPrefix + "MT2/Status", "TAG_AREA_REJECT");
    //   totalRejected++;
    //   return false;
    // }

    // =============== STANDARD DEVIATION MODEL ===============
    // Quadratic distance scaling, inversely linear with tag
    // count.
    // Formula: xyStdDev = coefficient * dist^2 / tagCount
    // This trusts close multi-tag observations strongly while being
    // appropriately uncertain for distant single-tag observations.
    double xyStdev =
        VisionConstants.XY_STDDEV_COEFFICIENT * Math.pow(avgTagDist, 2.0) / mt2.tagCount;
    xyStdev = Math.max(xyStdev, VisionConstants.XY_STDDEV_FLOOR);

    // Theta: trust multi-tag heading to gently correct gyro drift.
    // Single-tag: ignore heading (gyro is more reliable).
    // This is a key improvement - our old code used THETA_STDDEV=9999999 for
    // everything, meaning the gyro NEVER got corrected by vision. After collisions
    // or Pigeon drift, heading would diverge permanently.
    double thetaStdDev;
    if (mt2.tagCount >= 2) {
      thetaStdDev = VisionConstants.MULTI_TAG_THETA_STDDEV;
    } else {
      thetaStdDev = VisionConstants.SINGLE_TAG_THETA_STDDEV;
    }

    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdev, xyStdev, thetaStdDev);

    drivetrain.addVisionMeasurement(pose, mt2.timestampSeconds, stdDevs);

    totalMt2Accepted++;
    Logger.recordOutput(logPrefix + "MT2/Status", "ACCEPTED");
    Logger.recordOutput(logPrefix + "MT2/Pose", pose);
    Logger.recordOutput(logPrefix + "MT2/TagCount", mt2.tagCount);
    Logger.recordOutput(logPrefix + "MT2/AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "MT2/XYStdDev", xyStdev);
    Logger.recordOutput(logPrefix + "MT2/ThetaStdDev", thetaStdDev);
    return true;
  }

  private void tryAcceptMT1(LimelightHelpers.PoseEstimate mt1, Pose2d odoPose, String logPrefix) {

    if (mt1 == null || mt1.timestampSeconds == 0 || mt1.tagCount == 0) {
      Logger.recordOutput(logPrefix + "MT1/Status", "NO_DATA");
      totalRejected++;
      return;
    }

    Pose2d pose = mt1.pose;

    // SAFETY REMOVED: Accept all MT1 poses unconditionally
    // if (pose.equals(new Pose2d())) {
    //   Logger.recordOutput(logPrefix + "MT1/Status", "EMPTY_POSE");
    //   totalRejected++;
    //   return;
    // }

    // double x = pose.getX();
    // double y = pose.getY();
    // if (x < -FIELD_MARGIN
    //     || x > FIELD_LENGTH + FIELD_MARGIN
    //     || y < -FIELD_MARGIN
    //     || y > FIELD_WIDTH + FIELD_MARGIN) {
    //   Logger.recordOutput(logPrefix + "MT1/Status", "OUT_OF_BOUNDS");
    //   totalRejected++;
    //   return;
    // }

    // double headingDivergenceDeg =
    //     Math.abs(odoPose.getRotation().minus(pose.getRotation()).getDegrees());
    // if (headingDivergenceDeg > VisionConstants.HEADING_DIVERGENCE_THRESHOLD_DEG) {
    //   Logger.recordOutput(logPrefix + "MT1/Status", "HEADING_DIVERGE");
    //   totalRejected++;
    //   return;
    // }

    double avgTagDist = mt1.avgTagDist;

    // if (avgTagDist > VisionConstants.MAX_TAG_DISTANCE_METERS) {
    //   Logger.recordOutput(logPrefix + "MT1/Status", "DISTANCE_REJECT");
    //   totalRejected++;
    //   return;
    // }

    // Ambiguity check removed
    // if (mt1.tagCount == 1 && mt1.rawFiducials.length > 0) {
    //   double ambiguity = mt1.rawFiducials[0].ambiguity;
    //   Logger.recordOutput(logPrefix + "MT1/Ambiguity", ambiguity);
    //   if (ambiguity > VisionConstants.MT1_AMBIGUITY_THRESHOLD) {
    //     Logger.recordOutput(logPrefix + "MT1/Status", "AMBIGUITY_REJECT");
    //     totalRejected++;
    //     return;
    //   }
    // }

    // Pose-difference sanity check removed
    // double poseDifference = odoPose.getTranslation().getDistance(pose.getTranslation());
    // if (poseDifference > VisionConstants.MAX_POSE_DIFFERENCE_METERS) {
    //   Logger.recordOutput(logPrefix + "MT1/Status", "POSE_DIFF_REJECT");
    //   totalRejected++;
    //   return;
    // }

    // Standard deviation: quadratic distance scaling like 6328/1678.
    // MT1 is less reliable than MT2, so we use a higher coefficient (2x).
    double xyStdev = VisionConstants.XY_STDDEV_COEFFICIENT
        * 2.0
        * Math.pow(avgTagDist, 2.0)
        / (mt1.tagCount * mt1.tagCount);
    xyStdev = Math.max(xyStdev, VisionConstants.XY_STDDEV_FLOOR);

    // Theta: trust multi-tag MT1 heading weakly (like 254 with 6-30deg stddev).
    // Single-tag MT1 heading is unreliable - don't trust.
    double thetaStdDev;
    if (mt1.tagCount >= 2) {
      // ~0.5 rad = ~29deg
      thetaStdDev = VisionConstants.MULTI_TAG_THETA_STDDEV;
    } else {
      thetaStdDev = VisionConstants.SINGLE_TAG_THETA_STDDEV;
    }

    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdev, xyStdev, thetaStdDev);

    drivetrain.addVisionMeasurement(pose, mt1.timestampSeconds, stdDevs);

    totalMt1Accepted++;
    Logger.recordOutput(logPrefix + "MT1/Status", "ACCEPTED_FALLBACK");
    Logger.recordOutput(logPrefix + "MT1/Pose", pose);
    Logger.recordOutput(logPrefix + "MT1/TagCount", mt1.tagCount);
    Logger.recordOutput(logPrefix + "MT1/AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "MT1/XYStdDev", xyStdev);
    Logger.recordOutput(logPrefix + "MT1/ThetaStdDev", thetaStdDev);
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
