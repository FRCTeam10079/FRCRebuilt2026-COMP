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

  // ==================== JITTER DEBUGGING ====================
  // Track last accepted vision pose per camera to detect jumps between frames
  private final Map<String, Pose2d> lastAcceptedPose = new HashMap<>();
  private final Map<String, Double> lastAcceptedTimestamp = new HashMap<>();

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
    // boolean isAuto = RobotStateMachine.getInstance().getMatchState().autonomous;
    // Logger.recordOutput("Vision/AutoSkipped", isAuto);
    // if (isAuto) {
    // return;
    // }

    String[] names = VisionConstants.LIMELIGHT_NAMES;
    Pose2d odoPose = drivetrain.getState().Pose;
    ChassisSpeeds speeds = drivetrain.getState().Speeds;

    for (String name : names) {
      processCamera(name, odoPose, speeds);
    }

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
      Logger.recordOutput(logPrefix + "Debug/HasData", false);
      return;
    }

    Pose2d pose = mt1.pose;
    double avgTagDist = mt1.avgTagDist;

    // ---- Log ALL raw MT1 data before any filtering ----
    Logger.recordOutput(logPrefix + "Debug/HasData", true);
    Logger.recordOutput(logPrefix + "Debug/RawPose", pose);
    Logger.recordOutput(logPrefix + "Debug/RawTimestamp", mt1.timestampSeconds);
    Logger.recordOutput(logPrefix + "Debug/Latency", mt1.latency);
    Logger.recordOutput(logPrefix + "Debug/TagCount", mt1.tagCount);
    Logger.recordOutput(logPrefix + "Debug/TagSpan", mt1.tagSpan);
    Logger.recordOutput(logPrefix + "Debug/AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "Debug/AvgTagArea", mt1.avgTagArea);
    Logger.recordOutput(logPrefix + "Debug/IsMegaTag2", mt1.isMegaTag2);

    // Log age of vision measurement (how stale is this data?)
    double measurementAge = Timer.getFPGATimestamp() - mt1.timestampSeconds;
    Logger.recordOutput(logPrefix + "Debug/MeasurementAgeSec", measurementAge);

    // ---- Log per-tag raw fiducial data for jitter diagnosis ----
    int[] tagIds = new int[mt1.rawFiducials.length];
    double[] tagDistances = new double[mt1.rawFiducials.length];
    double[] tagAmbiguities = new double[mt1.rawFiducials.length];
    double[] tagAreas = new double[mt1.rawFiducials.length];
    for (int i = 0; i < mt1.rawFiducials.length; i++) {
      tagIds[i] = mt1.rawFiducials[i].id;
      tagDistances[i] = mt1.rawFiducials[i].distToRobot;
      tagAmbiguities[i] = mt1.rawFiducials[i].ambiguity;
      tagAreas[i] = mt1.rawFiducials[i].ta;
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

    // Single-tag: reject high-ambiguity
    if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
      double ambiguity = mt1.rawFiducials[0].ambiguity;
      Logger.recordOutput(logPrefix + "Debug/SingleTagAmbiguity", ambiguity);
      if (ambiguity > VisionConstants.MT1_AMBIGUITY_THRESHOLD) {
        Logger.recordOutput(logPrefix + "Status", "REJECTED_AMBIGUITY");
        Logger.recordOutput(
            logPrefix + "Debug/RejectionDetail",
            "ambiguity=" + String.format("%.3f", ambiguity)
                + " > threshold=" + VisionConstants.MT1_AMBIGUITY_THRESHOLD
                + " tagID=" + mt1.rawFiducials[0].id);
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
      Logger.recordOutput(logPrefix + "Debug/BootstrapDivergenceDeg", divergenceDeg);
      if (divergenceDeg > VisionConstants.HEADING_BOOTSTRAP_THRESHOLD_DEG) {
        Pose2d correctedPose = new Pose2d(odoPose.getTranslation(), pose.getRotation());
        drivetrain.resetPose(correctedPose);
        headingCorrections++;
        Logger.recordOutput(
            "Events/Vision/Last",
            "[Vision] One-shot heading bootstrap: "
                + String.format("%.1f", odoPose.getRotation().getDegrees())
                + "° -> "
                + String.format("%.1f", pose.getRotation().getDegrees())
                + "°");
        Logger.recordOutput("Events/Vision/Sequence", headingCorrections);
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
    double thetaStdDev = Double.POSITIVE_INFINITY;

    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdev, xyStdev, thetaStdDev);

    // ---- Log pre-fusion odometry pose (before Kalman filter update) ----
    Pose2d preFusionPose = drivetrain.getState().Pose;
    Logger.recordOutput(logPrefix + "Debug/PreFusionOdoPose", preFusionPose);

    drivetrain.addVisionMeasurement(pose, mt1.timestampSeconds, stdDevs);

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
      double timeBetween = mt1.timestampSeconds - prevTimestamp;
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
    lastAcceptedTimestamp.put(cameraName, mt1.timestampSeconds);

    totalAccepted++;
    Logger.recordOutput(logPrefix + "Status", "ACCEPTED");
    Logger.recordOutput(logPrefix + "Pose", pose);
    Logger.recordOutput(logPrefix + "TagCount", mt1.tagCount);
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
      LimelightHelpers.SetIMUMode(name, 1);

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
  }
}
