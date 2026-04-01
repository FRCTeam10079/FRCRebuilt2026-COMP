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
  private int loopCounter = 0;
  private boolean hasBootstrappedHeading = false;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    String[] names = VisionConstants.LIMELIGHT_NAMES;
    for (String name : names) {
      LimelightHelpers.setPipelineIndex(name, PIPELINE_APRILTAG);
      LimelightHelpers.setLEDMode_PipelineControl(name);
      LimelightHelpers.setLEDMode_ForceOff(name);

      LimelightHelpers.SetIMUMode(name, 3);
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

    double headingDeg = odoPose.getRotation().getDegrees();

    for (String name : names) {

      LimelightHelpers.SetRobotOrientation(name, headingDeg, 0, 0, 0, 0, 0);

      processCamera(name, odoPose, speeds);
    }

    // Print summary once per second (every 50 loops) - avoids roboRIO lag
    /*
     * if (loopCounter >= 50) {
     * System.out.println("[VISION] heading="
     * + String.format("%.1f", odoPose.getRotation().getDegrees())
     * + "deg accepted="
     * + totalAccepted
     * + " rejected="
     * + totalRejected
     * + " corrections="
     * + headingCorrections);
     * loopCounter = 0;
     * }
     */
    Logger.recordOutput("Vision/TotalAccepted", totalAccepted);
    Logger.recordOutput("Vision/TotalRejected", totalRejected);
    Logger.recordOutput("Vision/HeadingCorrections", headingCorrections);
  }

  private void processCamera(String cameraName, Pose2d odoPose, ChassisSpeeds speeds) {
    String logPrefix = "Vision/" + cameraName + "/";

    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    if (mt2 == null || mt2.timestampSeconds == 0 || mt2.tagCount == 0) {
      Logger.recordOutput(logPrefix + "Status", "NO_DATA");
      return;
    }

    Pose2d pose = mt2.pose;
    double avgTagDist = mt2.avgTagDist;

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
    // Single-tag: reject high-ambiguity
    if (mt2.tagCount == 1 && mt2.rawFiducials.length == 1) {
      if (mt2.rawFiducials[0].ambiguity > VisionConstants.MT1_AMBIGUITY_THRESHOLD) {
        Logger.recordOutput(logPrefix + "Status", "REJECTED_AMBIGUITY");
        totalRejected++;
        return;
      }
    }

    // ---- One-shot heading bootstrap ----
    // On first reliable multi-tag result, correct gyro if heading is way off.
    // This handles the Pigeon2 booting to 0° when the actual heading is ~180°.
    if (!hasBootstrappedHeading && mt2.tagCount >= 2) {
      double divergenceDeg = Math.abs(MathUtil.inputModulus(
          pose.getRotation().getDegrees() - odoPose.getRotation().getDegrees(), -180, 180));

      if (divergenceDeg > VisionConstants.HEADING_BOOTSTRAP_THRESHOLD_DEG) {
        Pose2d correctedPose = new Pose2d(odoPose.getTranslation(), pose.getRotation());
        drivetrain.resetPose(correctedPose);
        headingCorrections++;

        /*
         * System.out.println("[VISION] ONE-SHOT HEADING BOOTSTRAP: "
         * + String.format("%.1f", odoPose.getRotation().getDegrees())
         * + "deg -> "
         * + String.format("%.1f", pose.getRotation().getDegrees())
         * + "deg (divergence="
         * + String.format("%.1f", divergenceDeg)
         * + "deg, camera="
         * + cameraName
         * + ")");
         */
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
        / (mt2.tagCount * mt2.tagCount);

    // Never trust MT1 heading - coplanar tag ambiguity can flip it 180 degrees.
    // Gyro (Pigeon2) is the sole heading authority.
    // This matches Limelight's own official MT1 example:
    // VecBuilder.fill(.5,.5,9999999)
    double thetaStdDev = Double.POSITIVE_INFINITY;

    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdev, xyStdev, thetaStdDev);

    drivetrain.addVisionMeasurement(pose, mt2.timestampSeconds, stdDevs);

    totalAccepted++;
    Logger.recordOutput(logPrefix + "Status", "ACCEPTED");
    Logger.recordOutput(logPrefix + "Pose", pose);
    Logger.recordOutput(logPrefix + "TagCount", mt2.tagCount);
    Logger.recordOutput(logPrefix + "AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "XYStdDev", xyStdev);
    Logger.recordOutput(logPrefix + "ThetaStdDev", thetaStdDev);
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
