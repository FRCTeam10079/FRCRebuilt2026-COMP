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
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.util.HashMap;
import java.util.Map;
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

  // IMU mode transition: track last set mode to avoid spamming NT every loop.
  // -1 = not yet set.
  private int lastImuMode = -1;

  // Dashboard toggle: Elastic dashboard can flip this to disable vision fusion
  // in real-time. Published under /Robot/Vision/Enabled as a boolean entry
  // (read+write). Default: true (vision ON).
  private final BooleanEntry visionEnabledEntry =
      NetworkTableInstance.getDefault().getBooleanTopic("/Robot/Vision/Enabled").getEntry(true);

  // ==================== JITTER DEBUGGING ====================
  // Track last accepted vision pose per camera to detect jumps between frames
  private final Map<String, Pose2d> lastAcceptedPose = new HashMap<>();
  private final Map<String, Double> lastAcceptedTimestamp = new HashMap<>();

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Publish default value so the topic appears on Elastic immediately
    visionEnabledEntry.set(true);

    String[] names = VisionConstants.LIMELIGHT_NAMES;
    for (String name : names) {
      LimelightHelpers.setPipelineIndex(name, PIPELINE_APRILTAG);
      LimelightHelpers.setLEDMode_PipelineControl(name);
      LimelightHelpers.setLEDMode_ForceOff(name);
    }
  }

  @Override
  public void periodic() {
    boolean visionEnabled = visionEnabledEntry.get(true);
    Logger.recordOutput("Vision/Enabled", visionEnabled);

    String[] names = VisionConstants.LIMELIGHT_NAMES;
    Pose2d odoPose = drivetrain.getState().Pose;
    ChassisSpeeds speeds = drivetrain.getState().Speeds;
    double heading = odoPose.getRotation().getDegrees();

    // Feed heading to Limelights every frame (required for MegaTag2).
    // IMU mode 4 while enabled: LL4 internal IMU + external gyro assist.
    // Only send the mode change when it actually changes (avoid NT spam).
    for (String name : names) {
      LimelightHelpers.SetRobotOrientation(name, heading, 0, 0, 0, 0, 0);
      if (lastImuMode != 4) {
        LimelightHelpers.SetIMUMode(name, 4);
      }
    }
    lastImuMode = 4;

    if (!visionEnabled) {
      // Vision disabled from dashboard - log it and skip all processing.
      for (String name : names) {
        Logger.recordOutput("Vision/" + name + "/Status", "DISABLED_BY_DASHBOARD");
      }
      Logger.recordOutput("Vision/TotalAccepted", totalAccepted);
      Logger.recordOutput("Vision/TotalRejected", totalRejected);
      Logger.recordOutput("Vision/HeadingCorrections", headingCorrections);
      return;
    }

    for (String name : names) {
      processCamera(name, odoPose, speeds);
    }

    Logger.recordOutput("Vision/TotalAccepted", totalAccepted);
    Logger.recordOutput("Vision/TotalRejected", totalRejected);
    Logger.recordOutput("Vision/HeadingCorrections", headingCorrections);
  }

  private void processCamera(String cameraName, Pose2d odoPose, ChassisSpeeds speeds) {
    String logPrefix = "Vision/" + cameraName + "/";

    // MegaTag2: uses robot heading (from SetRobotOrientation) to eliminate
    // the coplanar pose ambiguity problem. Dramatically more stable than MT1
    // for single-tag observations.
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    if (mt2 == null || mt2.timestampSeconds == 0 || mt2.tagCount == 0) {
      Logger.recordOutput(logPrefix + "Status", "NO_DATA");
      Logger.recordOutput(logPrefix + "Debug/HasData", false);
      return;
    }

    Pose2d pose = mt2.pose;
    double avgTagDist = mt2.avgTagDist;

    // ---- Log ALL raw data before any filtering ----
    Logger.recordOutput(logPrefix + "Debug/HasData", true);
    Logger.recordOutput(logPrefix + "Debug/RawPose", pose);
    Logger.recordOutput(logPrefix + "Debug/RawTimestamp", mt2.timestampSeconds);
    Logger.recordOutput(logPrefix + "Debug/Latency", mt2.latency);
    Logger.recordOutput(logPrefix + "Debug/TagCount", mt2.tagCount);
    Logger.recordOutput(logPrefix + "Debug/TagSpan", mt2.tagSpan);
    Logger.recordOutput(logPrefix + "Debug/AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "Debug/AvgTagArea", mt2.avgTagArea);
    Logger.recordOutput(logPrefix + "Debug/IsMegaTag2", mt2.isMegaTag2);

    // Log age of vision measurement (how stale is this data?)
    double measurementAge = Timer.getFPGATimestamp() - mt2.timestampSeconds;
    Logger.recordOutput(logPrefix + "Debug/MeasurementAgeSec", measurementAge);

    // ---- Log per-tag raw fiducial data for jitter diagnosis ----
    int[] tagIds = new int[mt2.rawFiducials.length];
    double[] tagDistances = new double[mt2.rawFiducials.length];
    double[] tagAmbiguities = new double[mt2.rawFiducials.length];
    double[] tagAreas = new double[mt2.rawFiducials.length];
    for (int i = 0; i < mt2.rawFiducials.length; i++) {
      tagIds[i] = mt2.rawFiducials[i].id;
      tagDistances[i] = mt2.rawFiducials[i].distToRobot;
      tagAmbiguities[i] = mt2.rawFiducials[i].ambiguity;
      tagAreas[i] = mt2.rawFiducials[i].ta;
    }
    Logger.recordOutput(logPrefix + "Debug/TagIDs", tagIds);
    Logger.recordOutput(logPrefix + "Debug/TagDistances", tagDistances);
    Logger.recordOutput(logPrefix + "Debug/TagAmbiguities", tagAmbiguities);
    Logger.recordOutput(logPrefix + "Debug/TagAreas", tagAreas);

    // ---- Log vision-vs-odometry delta (how far is vision pulling the pose?) ----
    double visionOdoDeltaX = pose.getX() - odoPose.getX();
    double visionOdoDeltaY = pose.getY() - odoPose.getY();
    double visionOdoDeltaDistM = Math.hypot(visionOdoDeltaX, visionOdoDeltaY);
    double visionOdoDeltaHeadingDeg = Math.abs(MathUtil.inputModulus(
        pose.getRotation().getDegrees() - odoPose.getRotation().getDegrees(), -180, 180));
    Logger.recordOutput(logPrefix + "Debug/VisionVsOdo/DeltaX", visionOdoDeltaX);
    Logger.recordOutput(logPrefix + "Debug/VisionVsOdo/DeltaY", visionOdoDeltaY);
    Logger.recordOutput(logPrefix + "Debug/VisionVsOdo/DeltaDistM", visionOdoDeltaDistM);
    Logger.recordOutput(logPrefix + "Debug/VisionVsOdo/DeltaHeadingDeg", visionOdoDeltaHeadingDeg);

    // ---- Rejection checks ----

    // Reject if robot is spinning too fast for reliable vision
    double omegaDegPerSec = Math.toDegrees(Math.abs(speeds.omegaRadiansPerSecond));
    Logger.recordOutput(logPrefix + "Debug/OmegaDegPerSec", omegaDegPerSec);
    if (omegaDegPerSec > VisionConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
      Logger.recordOutput(logPrefix + "Status", "REJECTED_ANGULAR_VELOCITY");
      Logger.recordOutput(
          logPrefix + "Debug/RejectionDetail",
          "omega=" + String.format("%.1f", omegaDegPerSec) + " > threshold="
              + VisionConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC);
      totalRejected++;
      return;
    }

    // Pose outside field bounds (with margin) = clearly wrong
    if (pose.getX() < -FIELD_MARGIN
        || pose.getX() > FIELD_LENGTH + FIELD_MARGIN
        || pose.getY() < -FIELD_MARGIN
        || pose.getY() > FIELD_WIDTH + FIELD_MARGIN) {
      Logger.recordOutput(logPrefix + "Status", "REJECTED_OUT_OF_FIELD");
      Logger.recordOutput(
          logPrefix + "Debug/RejectionDetail",
          "pose=(" + String.format("%.2f", pose.getX()) + ", " + String.format("%.2f", pose.getY())
              + ") outside field bounds");
      totalRejected++;
      return;
    }

    // Reject if vision pose is too far from odometry (bogus measurement)
    if (visionOdoDeltaDistM > VisionConstants.MAX_POSE_JUMP_METERS) {
      Logger.recordOutput(logPrefix + "Status", "REJECTED_POSE_JUMP");
      Logger.recordOutput(
          logPrefix + "Debug/RejectionDetail",
          "visionVsOdoDist=" + String.format("%.3f", visionOdoDeltaDistM) + "m > threshold="
              + VisionConstants.MAX_POSE_JUMP_METERS + "m");
      totalRejected++;
      return;
    }

    // Single-tag quality gate: distance only.
    // MT2 eliminates the coplanar ambiguity problem, so hard-rejecting on
    // ambiguity is unnecessary and counterproductive. Ambiguity is still used
    // as a soft stddev scaling factor below.
    if (mt2.tagCount == 1 && mt2.rawFiducials.length == 1) {
      // Log ambiguity for diagnostics even though we don't hard-reject on it
      Logger.recordOutput(logPrefix + "Debug/SingleTagAmbiguity", mt2.rawFiducials[0].ambiguity);

      // Reject far single-tag: noise increases dramatically beyond 4m
      if (avgTagDist > VisionConstants.SINGLE_TAG_MAX_DISTANCE_METERS) {
        Logger.recordOutput(logPrefix + "Status", "REJECTED_SINGLE_TAG_DISTANCE");
        Logger.recordOutput(
            logPrefix + "Debug/RejectionDetail",
            "singleTagDist=" + String.format("%.2f", avgTagDist)
                + "m > threshold=" + VisionConstants.SINGLE_TAG_MAX_DISTANCE_METERS
                + "m tagID=" + mt2.rawFiducials[0].id);
        totalRejected++;
        return;
      }
    }

    // ---- One-shot heading bootstrap (MT1-based) ----
    // MT2 heading comes from the gyro, so it can't detect heading errors.
    // Use an MT1 query to check if the Pigeon2 heading is grossly wrong.
    // This fires once on the first multi-tag result, then never again.
    if (!hasBootstrappedHeading && mt2.tagCount >= 2) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
      if (mt1 != null && mt1.tagCount >= 2) {
        double divergenceDeg = Math.abs(MathUtil.inputModulus(
            mt1.pose.getRotation().getDegrees() - odoPose.getRotation().getDegrees(), -180, 180));
        Logger.recordOutput(logPrefix + "Debug/BootstrapDivergenceDeg", divergenceDeg);
        if (divergenceDeg > VisionConstants.HEADING_BOOTSTRAP_THRESHOLD_DEG) {
          Pose2d correctedPose = new Pose2d(odoPose.getTranslation(), mt1.pose.getRotation());
          drivetrain.resetPose(correctedPose);
          headingCorrections++;
          Logger.recordOutput(
              "Events/Vision/Last",
              "[Vision] One-shot heading bootstrap: "
                  + String.format("%.1f", odoPose.getRotation().getDegrees())
                  + "° -> "
                  + String.format("%.1f", mt1.pose.getRotation().getDegrees())
                  + "°");
          Logger.recordOutput("Events/Vision/Sequence", headingCorrections);
          Logger.recordOutput(logPrefix + "HeadingBootstrap", true);
        }
      }
      hasBootstrappedHeading = true;
    }

    // ---- Standard deviation model ----
    // Base: dist^1.5 scaling, inversely proportional to tagCount^2.
    double xyStdev = VisionConstants.XY_STDDEV_COEFFICIENT
        * Math.pow(avgTagDist, VisionConstants.XY_STDDEV_EXPONENT)
        / (mt2.tagCount * mt2.tagCount);

    // Single-tag penalty: even with MT2 (no ambiguity problem), single-tag
    // observations have 2.5-3x more noise than multi-tag per log analysis.
    if (mt2.tagCount == 1) {
      xyStdev *= VisionConstants.SINGLE_TAG_STDDEV_MULTIPLIER;
    }

    // Ambiguity-scaled trust: continuously degrade trust as ambiguity rises.
    double maxAmbiguity = 0.0;
    for (var fid : mt2.rawFiducials) {
      maxAmbiguity = Math.max(maxAmbiguity, fid.ambiguity);
    }
    xyStdev *= (1.0 + maxAmbiguity * VisionConstants.AMBIGUITY_STDDEV_SCALE);

    // Divergence-based graduated trust: inflate stddev proportionally to
    // vision-odometry distance. This replaces the old hard 2.0m pose-jump
    // reject that caused death spirals. Large corrections are still possible
    // but happen slowly; as the pose converges, trust automatically increases.
    double divergenceInflation = 1.0
        + VisionConstants.DIVERGENCE_STDDEV_SCALE
            * Math.max(0, visionOdoDeltaDistM - VisionConstants.DIVERGENCE_RAMP_START_METERS);
    xyStdev *= divergenceInflation;

    // Safety clamp: ensure stddev is a valid finite positive number.
    // Prevents NaN/Infinity from crashing the WPILib Kalman filter.
    // Cap at 10.0 so anything above ~1.0 is already "barely trusted";
    // 10.0 means the measurement is essentially ignored without hard-rejecting.
    if (!Double.isFinite(xyStdev) || xyStdev <= 0) {
      xyStdev = 10.0;
    } else {
      xyStdev = Math.min(xyStdev, 10.0);
    }

    Logger.recordOutput(logPrefix + "Debug/MaxAmbiguity", maxAmbiguity);
    Logger.recordOutput(logPrefix + "Debug/DivergenceInflation", divergenceInflation);
    Logger.recordOutput(logPrefix + "Debug/FinalXYStdDev", xyStdev);

    // MT2 heading = gyro heading (redundant). Never trust vision heading.
    double thetaStdDev = Double.POSITIVE_INFINITY;

    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdev, xyStdev, thetaStdDev);

    // ---- Log pre-fusion odometry pose (before Kalman filter update) ----
    Pose2d preFusionPose = drivetrain.getState().Pose;
    Logger.recordOutput(logPrefix + "Debug/PreFusionOdoPose", preFusionPose);

    drivetrain.addVisionMeasurement(pose, mt2.timestampSeconds, stdDevs);

    // ---- Log post-fusion odometry pose (after Kalman filter update) ----
    Pose2d postFusionPose = drivetrain.getState().Pose;
    Logger.recordOutput(logPrefix + "Debug/PostFusionOdoPose", postFusionPose);

    // ---- Log the Kalman filter correction magnitude ----
    double fusionCorrectionX = postFusionPose.getX() - preFusionPose.getX();
    double fusionCorrectionY = postFusionPose.getY() - preFusionPose.getY();
    double fusionCorrectionDist = Math.hypot(fusionCorrectionX, fusionCorrectionY);
    Logger.recordOutput(logPrefix + "Debug/FusionCorrection/DeltaX", fusionCorrectionX);
    Logger.recordOutput(logPrefix + "Debug/FusionCorrection/DeltaY", fusionCorrectionY);
    Logger.recordOutput(logPrefix + "Debug/FusionCorrection/DeltaDistM", fusionCorrectionDist);

    // ---- Jitter detection: consecutive accepted pose jump ----
    Pose2d prevAccepted = lastAcceptedPose.get(cameraName);
    Double prevTimestamp = lastAcceptedTimestamp.get(cameraName);
    if (prevAccepted != null && prevTimestamp != null) {
      double jumpX = pose.getX() - prevAccepted.getX();
      double jumpY = pose.getY() - prevAccepted.getY();
      double jumpDist = Math.hypot(jumpX, jumpY);
      double jumpHeadingDeg = Math.abs(MathUtil.inputModulus(
          pose.getRotation().getDegrees() - prevAccepted.getRotation().getDegrees(), -180, 180));
      double timeBetween = mt2.timestampSeconds - prevTimestamp;
      Logger.recordOutput(logPrefix + "Debug/Jitter/JumpX", jumpX);
      Logger.recordOutput(logPrefix + "Debug/Jitter/JumpY", jumpY);
      Logger.recordOutput(logPrefix + "Debug/Jitter/JumpDistM", jumpDist);
      Logger.recordOutput(logPrefix + "Debug/Jitter/JumpHeadingDeg", jumpHeadingDeg);
      Logger.recordOutput(logPrefix + "Debug/Jitter/TimeBetweenSec", timeBetween);
      // Velocity of pose jump (m/s) - unrealistically high = jitter
      if (timeBetween > 0.001) {
        Logger.recordOutput(logPrefix + "Debug/Jitter/ImpliedVelocityMps", jumpDist / timeBetween);
      }
    }
    lastAcceptedPose.put(cameraName, pose);
    lastAcceptedTimestamp.put(cameraName, mt2.timestampSeconds);

    totalAccepted++;
    Logger.recordOutput(logPrefix + "Status", "ACCEPTED");
    Logger.recordOutput(logPrefix + "Pose", pose);
    Logger.recordOutput(logPrefix + "TagCount", mt2.tagCount);
    Logger.recordOutput(logPrefix + "AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "XYStdDev", xyStdev);
    Logger.recordOutput(logPrefix + "ThetaStdDev", thetaStdDev);
    Logger.recordOutput(logPrefix + "Debug/RejectionDetail", "NONE");
  }

  public void updateWhileDisabled() {
    String[] names = VisionConstants.LIMELIGHT_NAMES;
    Pose2d currentPose = drivetrain.getState().Pose;
    double currentHeadingDeg = currentPose.getRotation().getDegrees();
    /*
     * System.out.println("[VISION-DEBUG] updateWhileDisabled() | gyroHeading="
     * + String.format("%.1f", currentHeadingDeg)
     * + "deg | pose=("
     * + String.format("%.2f", currentPose.getX())
     * + ", "
     * + String.format("%.2f", currentPose.getY())
     * + ") | MT1_HEADING_CORRECTION_ENABLED="
     * + VisionConstants.USE_MT1_HEADING_CORRECTION_WHILE_DISABLED);
     */

    for (String name : names) {
      LimelightHelpers.SetRobotOrientation(name, currentHeadingDeg, 0, 0, 0, 0, 0);
      // Mode 1 while disabled: seeds LL4's internal IMU with external gyro.
      if (lastImuMode != 1) {
        LimelightHelpers.SetIMUMode(name, 1);
      }

      if (VisionConstants.USE_MT1_HEADING_CORRECTION_WHILE_DISABLED) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (mt1.tagCount >= 2 && mt1.timestampSeconds != 0) {
          double mt1HeadingDeg = mt1.pose.getRotation().getDegrees();
          double divergenceDeg =
              Math.abs(MathUtil.inputModulus(mt1HeadingDeg - currentHeadingDeg, -180, 180));
          /*
           * System.out.println("[VISION-DEBUG] [DISABLED] ["
           * + name
           * + "] MT1 multi-tag heading="
           * + String.format("%.1f", mt1HeadingDeg)
           * + "deg gyroHeading="
           * + String.format("%.1f", currentHeadingDeg)
           * + "deg divergence="
           * + String.format("%.1f", divergenceDeg)
           * + "deg threshold="
           * + VisionConstants.MT1_HEADING_CORRECTION_THRESHOLD_DEG
           * + "deg");
           */
          Logger.recordOutput("Vision/" + name + "/Disabled/MT1HeadingDeg", mt1HeadingDeg);
          Logger.recordOutput("Vision/" + name + "/Disabled/HeadingDivergenceDeg", divergenceDeg);

          if (divergenceDeg > VisionConstants.MT1_HEADING_CORRECTION_THRESHOLD_DEG) {
            Pose2d correctedPose = new Pose2d(currentPose.getTranslation(), mt1.pose.getRotation());
            /*
             * System.out.println("[VISION-DEBUG] [DISABLED] ["
             * + name
             * + "] !!! HEADING CORRECTION FIRING !!! resetting pose heading from "
             * + String.format("%.1f", currentHeadingDeg)
             * + "deg -> "
             * + String.format("%.1f", mt1HeadingDeg)
             * + "deg");
             */
            drivetrain.resetPose(correctedPose);

            hasBootstrappedHeading = true;
            headingCorrections++;
            Logger.recordOutput(
                "Events/Vision/Last",
                "[Vision] Auto-corrected heading from MT1 multi-tag: "
                    + String.format("%.1f", currentHeadingDeg)
                    + "° -> "
                    + String.format("%.1f", mt1HeadingDeg)
                    + "° (divergence: "
                    + String.format("%.1f", divergenceDeg)
                    + "°, camera: "
                    + name
                    + ")");
            Logger.recordOutput("Events/Vision/Sequence", headingCorrections);
            Logger.recordOutput("Vision/" + name + "/Disabled/HeadingCorrected", true);

            break;
          }
        }
      }
    }
    lastImuMode = 1;
  }
}
