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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem using dual Limelight 4 cameras for AprilTag-based robot localization.
 *
 * <p>Architecture:
 *
 * <ul>
 *   <li><b>Primary:</b> MegaTag2 (MT2) — single-tag ambiguity-free, requires external heading. Uses
 *       IMU Mode 0 (EXTERNAL_ONLY) so the Pigeon2 heading is sent directly via
 *       SetRobotOrientation() every frame. No LL4 internal IMU involvement.
 *   <li><b>Fallback:</b> MegaTag1 (MT1) — multi-tag geometric solver. When MT1 sees 2+ tags, the
 *       heading is computed geometrically and is independent of the gyro. Provides both pose
 *       fallback (when MT2 is rejected) and heading validation/correction.
 * </ul>
 *
 * <p>Both cameras feed into the CTRE SwerveDrivetrain's built-in Kalman filter pose estimator with
 * distance-scaled standard deviations. No alliance-based tag filtering is applied — all visible
 * tags contribute to localization.
 *
 * <p>Key design decisions:
 *
 * <ul>
 *   <li>IMU Mode 0: Eliminates LL4 internal IMU drift failures entirely.
 *   <li>Angular velocity passed via SetRobotOrientation() for motion compensation.
 *   <li>MT1 multi-tag heading used to auto-correct gyro drift while disabled.
 *   <li>Heading divergence check: rejects MT2 if its heading disagrees with pose estimator.
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

  // ==================== CONFIGURATION ====================

  private static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC =
      VisionConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC;

  private static final double FIELD_MARGIN = VisionConstants.FIELD_BORDER_MARGIN;
  private static final double FIELD_LENGTH = VisionConstants.FIELD_LENGTH_METERS;
  private static final double FIELD_WIDTH = VisionConstants.FIELD_WIDTH_METERS;

  private static final int PIPELINE_APRILTAG = 0;
  private static final int PIPELINE_DISABLED = 1;

  // ==================== HARDWARE REFERENCES ====================

  private final CommandSwerveDrivetrain drivetrain;

  // Per-camera NetworkTable entries for targeting data (used by AlignToAprilTag)
  private final NetworkTableEntry[] tvEntries;
  private final NetworkTableEntry[] tidEntries;
  private final NetworkTableEntry[] txEntries;
  private final NetworkTableEntry[] tyEntries;
  private final NetworkTableEntry[] taEntries;

  /** Index of the camera currently providing the best target (largest ta). */
  private int activeCameraIndex = 0;

  // ==================== TELEMETRY ====================
  private int totalMt2Accepted = 0;
  private int totalMt1Accepted = 0;
  private int totalRejected = 0;
  private int headingCorrections = 0;

  /**
   * Creates a new VisionSubsystem.
   *
   * @param drivetrain The CommandSwerveDrivetrain to feed vision measurements into.
   */
  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    String[] names = VisionConstants.LIMELIGHT_NAMES;
    tvEntries = new NetworkTableEntry[names.length];
    tidEntries = new NetworkTableEntry[names.length];
    txEntries = new NetworkTableEntry[names.length];
    tyEntries = new NetworkTableEntry[names.length];
    taEntries = new NetworkTableEntry[names.length];

    for (int i = 0; i < names.length; i++) {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(names[i]);
      tvEntries[i] = table.getEntry("tv");
      tidEntries[i] = table.getEntry("tid");
      txEntries[i] = table.getEntry("tx");
      tyEntries[i] = table.getEntry("ty");
      taEntries[i] = table.getEntry("ta");

      // Configure pipelines and LEDs
      LimelightHelpers.setPipelineIndex(names[i], PIPELINE_APRILTAG);
      LimelightHelpers.setLEDMode_PipelineControl(names[i]);
      LimelightHelpers.setLEDMode_ForceOff(names[i]);

      // IMU Mode 0: EXTERNAL_ONLY — Pigeon2 heading sent directly, no LL4 internal
      // IMU.
      // This eliminates the class of drift/seeding failures observed with Mode 4.
      LimelightHelpers.SetIMUMode(names[i], 0);
    }
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    // Update active camera selection (for targeting data getters)
    updateActiveCameraSelection();

    // Process both cameras for pose estimation
    String[] names = VisionConstants.LIMELIGHT_NAMES;
    Pose2d odoPose = drivetrain.getState().Pose;
    double fusedHeadingDeg = odoPose.getRotation().getDegrees();
    double angularVelocityDegPerSec =
        drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();

    for (int i = 0; i < names.length; i++) {
      processCamera(names[i], odoPose, fusedHeadingDeg, angularVelocityDegPerSec);
    }

    // Log global telemetry
    Logger.recordOutput("Vision/TotalMT2Accepted", totalMt2Accepted);
    Logger.recordOutput("Vision/TotalMT1Accepted", totalMt1Accepted);
    Logger.recordOutput("Vision/TotalRejected", totalRejected);
    Logger.recordOutput("Vision/HeadingCorrections", headingCorrections);
    Logger.recordOutput("Vision/ActiveCamera", VisionConstants.LIMELIGHT_NAMES[activeCameraIndex]);
    Logger.recordOutput("Vision/FusedHeadingDeg", fusedHeadingDeg);
    Logger.recordOutput(
        "Vision/Pigeon2RawHeadingDeg", drivetrain.getPigeon2().getYaw().getValueAsDouble());
  }

  // ==================== CORE VISION PIPELINE ====================

  /**
   * Process a single camera's MegaTag2 + MegaTag1 estimates through the validation chain and feed
   * accepted measurements into the drivetrain's pose estimator.
   *
   * <p>Pipeline:
   *
   * <ol>
   *   <li>Send heading + angular velocity to Limelight (required for MT2)
   *   <li>Read both MT2 and MT1 estimates
   *   <li>Try MT2 first (primary) through validation gates
   *   <li>If MT2 rejected, try MT1 as fallback with higher std devs
   * </ol>
   */
  private void processCamera(
      String cameraName, Pose2d odoPose, double fusedHeadingDeg, double angularVelocityDegPerSec) {

    String logPrefix = "Vision/" + cameraName + "/";

    // Step 1: Send current heading + angular velocity to Limelight
    // The yawRate parameter helps MT2 compensate for heading change between
    // frame capture and NetworkTables update (~20ms delay at 50Hz).
    LimelightHelpers.SetRobotOrientation(
        cameraName, fusedHeadingDeg, angularVelocityDegPerSec, 0, 0, 0, 0);

    // Step 2: Read both MT2 and MT1 estimates
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    // Step 3: Try MT2 (primary)
    boolean mt2Accepted =
        tryAcceptMT2(mt2, cameraName, odoPose, angularVelocityDegPerSec, logPrefix);

    // Step 4: If MT2 failed, try MT1 as fallback
    if (!mt2Accepted) {
      tryAcceptMT1(mt1, cameraName, odoPose, angularVelocityDegPerSec, logPrefix);
    }

    // Step 5: Log MT1 heading for diagnostic comparison (even if MT2 was accepted)
    logMT1HeadingDiagnostics(mt1, odoPose, logPrefix);
  }

  /**
   * Attempt to validate and accept a MegaTag2 estimate.
   *
   * @return true if the estimate was accepted, false if rejected
   */
  private boolean tryAcceptMT2(
      LimelightHelpers.PoseEstimate mt2,
      String cameraName,
      Pose2d odoPose,
      double angularVelocityDegPerSec,
      String logPrefix) {

    // Gate 1: Null / stale / no tags
    if (mt2 == null || mt2.timestampSeconds == 0 || mt2.tagCount == 0) {
      Logger.recordOutput(logPrefix + "MT2/Status", "NO_DATA");
      return false;
    }

    // Gate 2: Angular velocity — MT2 degrades when spinning fast
    if (Math.abs(angularVelocityDegPerSec) > MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
      Logger.recordOutput(logPrefix + "MT2/Status", "ANG_VEL_REJECT");
      totalRejected++;
      return false;
    }

    // Gate 3: Field bounds — reject poses outside the field
    double x = mt2.pose.getX();
    double y = mt2.pose.getY();
    if (x < -FIELD_MARGIN
        || x > FIELD_LENGTH + FIELD_MARGIN
        || y < -FIELD_MARGIN
        || y > FIELD_WIDTH + FIELD_MARGIN) {
      Logger.recordOutput(logPrefix + "MT2/Status", "OUT_OF_BOUNDS");
      totalRejected++;
      return false;
    }

    // Gate 4: Heading divergence check — catch bad heading input to MT2.
    // MT2 uses the heading we send it, so if the pose estimator heading is wrong,
    // MT2 will produce wrong XY. Compare MT2's reported rotation (which should
    // roughly match what we sent) against our current pose estimator heading.
    double headingDivergenceDeg = Math.abs(MathUtil.inputModulus(
        mt2.pose.getRotation().getDegrees() - odoPose.getRotation().getDegrees(), -180, 180));
    Logger.recordOutput(logPrefix + "MT2/HeadingDivergenceDeg", headingDivergenceDeg);

    if (headingDivergenceDeg > VisionConstants.HEADING_DIVERGENCE_THRESHOLD_DEG) {
      Logger.recordOutput(logPrefix + "MT2/Status", "HEADING_DIVERGE");
      totalRejected++;
      return false;
    }

    // Compute distance-scaled standard deviations
    // base_stddev × avgTagDist — farther tags get trusted less
    double avgTagDist = mt2.avgTagDist;
    double scaledXY = VisionConstants.MT2_BASE_XY_STDDEV * avgTagDist;

    // Multi-tag bonus: if MT2 sees 2+ tags, trust it more
    if (mt2.tagCount >= 2) {
      scaledXY *= 0.5;
    }

    Matrix<N3, N1> stdDevs = VecBuilder.fill(scaledXY, scaledXY, VisionConstants.MT2_THETA_STDDEV);

    // Feed into the CTRE pose estimator's Kalman filter
    drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);

    // Telemetry
    totalMt2Accepted++;
    Logger.recordOutput(logPrefix + "MT2/Status", "ACCEPTED");
    Logger.recordOutput(logPrefix + "MT2/Pose", mt2.pose);
    Logger.recordOutput(logPrefix + "MT2/TagCount", mt2.tagCount);
    Logger.recordOutput(logPrefix + "MT2/AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "MT2/XYStdDev", scaledXY);
    double odoDivergence = mt2.pose.getTranslation().getDistance(odoPose.getTranslation());
    Logger.recordOutput(logPrefix + "MT2/OdoDivergence", odoDivergence);
    if (odoDivergence > VisionConstants.POSE_DIVERGENCE_WARNING_METERS) {
      Logger.recordOutput(logPrefix + "MT2/Warning", "HIGH_DIVERGENCE");
    }
    return true;
  }

  /**
   * Attempt to validate and accept a MegaTag1 estimate as a fallback. MT1 is used when MT2 was
   * rejected (spinning, out-of-bounds, heading divergence, etc.).
   *
   * <p>With 2+ tags, MT1 provides geometrically computed heading — useful because it doesn't depend
   * on external heading input. With 1 tag, MT1 has pose ambiguity so we use very high std devs.
   */
  private void tryAcceptMT1(
      LimelightHelpers.PoseEstimate mt1,
      String cameraName,
      Pose2d odoPose,
      double angularVelocityDegPerSec,
      String logPrefix) {

    // Gate: Null / stale / no tags
    if (mt1 == null || mt1.timestampSeconds == 0 || mt1.tagCount == 0) {
      Logger.recordOutput(logPrefix + "MT1/Status", "NO_DATA");
      totalRejected++;
      return;
    }

    // Gate: Field bounds
    double x = mt1.pose.getX();
    double y = mt1.pose.getY();
    if (x < -FIELD_MARGIN
        || x > FIELD_LENGTH + FIELD_MARGIN
        || y < -FIELD_MARGIN
        || y > FIELD_WIDTH + FIELD_MARGIN) {
      Logger.recordOutput(logPrefix + "MT1/Status", "OUT_OF_BOUNDS");
      totalRejected++;
      return;
    }

    // Compute std devs based on tag count
    double avgTagDist = mt1.avgTagDist;
    double xyStdDev;
    double thetaStdDev;

    if (mt1.tagCount >= 2) {
      // Multi-tag MT1: heading is geometric, trust it
      xyStdDev = VisionConstants.MT1_MULTI_TAG_XY_STDDEV * avgTagDist;
      thetaStdDev = VisionConstants.MT1_MULTI_TAG_THETA_STDDEV;
    } else {
      // Single-tag MT1: heading is ambiguous, high std devs, distrust theta entirely
      xyStdDev = VisionConstants.MT1_SINGLE_TAG_XY_STDDEV * avgTagDist;
      thetaStdDev = VisionConstants.MT1_SINGLE_TAG_THETA_STDDEV;
    }

    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

    // Feed into pose estimator
    drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds, stdDevs);

    totalMt1Accepted++;
    Logger.recordOutput(logPrefix + "MT1/Status", "ACCEPTED_FALLBACK");
    Logger.recordOutput(logPrefix + "MT1/Pose", mt1.pose);
    Logger.recordOutput(logPrefix + "MT1/TagCount", mt1.tagCount);
    Logger.recordOutput(logPrefix + "MT1/AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "MT1/XYStdDev", xyStdDev);
    Logger.recordOutput(logPrefix + "MT1/ThetaStdDev", thetaStdDev);
  }

  /**
   * Log MT1 heading vs pose estimator heading for diagnostics. This runs even when MT2 was
   * accepted, providing constant visibility into whether the heading is correct.
   */
  private void logMT1HeadingDiagnostics(
      LimelightHelpers.PoseEstimate mt1, Pose2d odoPose, String logPrefix) {
    if (mt1 == null || mt1.tagCount < 2) {
      return; // Only trust MT1 heading with 2+ tags
    }

    double mt1HeadingDeg = mt1.pose.getRotation().getDegrees();
    double odoHeadingDeg = odoPose.getRotation().getDegrees();
    double divergenceDeg =
        Math.abs(MathUtil.inputModulus(mt1HeadingDeg - odoHeadingDeg, -180, 180));

    Logger.recordOutput(logPrefix + "MT1/HeadingDeg", mt1HeadingDeg);
    Logger.recordOutput(logPrefix + "MT1/HeadingDivergenceDeg", divergenceDeg);
  }

  // ==================== DISABLED PERIOD: HEADING AUTO-CORRECTION
  // ====================

  /**
   * Called continuously while disabled (pre-match and between auto/teleop). Uses IMU Mode 0 and
   * sends robot heading to the Limelights.
   *
   * <p>Also checks MT1 multi-tag heading against the pose estimator. If they diverge significantly,
   * auto-corrects the pose estimator heading. This prevents the "wrong heading -> wrong MT2 ->
   * stuck forever" death spiral.
   */
  public void updateWhileDisabled() {
    String[] names = VisionConstants.LIMELIGHT_NAMES;
    Pose2d currentPose = drivetrain.getState().Pose;
    double currentHeadingDeg = currentPose.getRotation().getDegrees();

    for (String name : names) {
      // Always keep sending heading and keep Mode 0
      LimelightHelpers.SetRobotOrientation(name, currentHeadingDeg, 0, 0, 0, 0, 0);
      LimelightHelpers.SetIMUMode(name, 0); // EXTERNAL_ONLY

      // MT1 heading auto-correction while disabled
      if (VisionConstants.USE_MT1_HEADING_CORRECTION_WHILE_DISABLED) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (mt1 != null && mt1.tagCount >= 2 && mt1.timestampSeconds != 0) {
          double mt1HeadingDeg = mt1.pose.getRotation().getDegrees();
          double divergenceDeg =
              Math.abs(MathUtil.inputModulus(mt1HeadingDeg - currentHeadingDeg, -180, 180));

          Logger.recordOutput("Vision/" + name + "/Disabled/MT1HeadingDeg", mt1HeadingDeg);
          Logger.recordOutput("Vision/" + name + "/Disabled/HeadingDivergenceDeg", divergenceDeg);

          if (divergenceDeg > VisionConstants.MT1_HEADING_CORRECTION_THRESHOLD_DEG) {
            // MT1 multi-tag heading disagrees with our pose estimator heading.
            // Re-seed the pose estimator with the MT1 heading (keep XY).
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

            // Once corrected, break to avoid conflicting corrections from the other camera
            break;
          }
        }
      }
    }
  }

  // ==================== PIPELINE CONTROL ====================

  /** Enable AprilTag pipeline on both cameras. */
  public void enable() {
    for (String name : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.setPipelineIndex(name, PIPELINE_APRILTAG);
    }
  }

  /** Disable vision processing on both cameras to reduce CPU/network load. */
  public void disable() {
    for (String name : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.setPipelineIndex(name, PIPELINE_DISABLED);
    }
  }

  // ==================== ACTIVE CAMERA SELECTION ====================

  /**
   * Select the camera with the best (largest area) target. Used by targeting getters (getTid,
   * getTx, etc.) for alignment commands.
   */
  private void updateActiveCameraSelection() {
    String[] names = VisionConstants.LIMELIGHT_NAMES;
    boolean[] valid = new boolean[names.length];
    double[] areas = new double[names.length];

    for (int i = 0; i < names.length; i++) {
      valid[i] = tvEntries[i].getDouble(0) == 1.0 && ((int) tidEntries[i].getDouble(0)) != 0;
      areas[i] = taEntries[i].getDouble(0);
    }

    // Pick camera with largest target area among valid cameras
    int bestIndex = 0;
    double bestArea = -1;
    boolean anyValid = false;

    for (int i = 0; i < names.length; i++) {
      if (valid[i] && areas[i] > bestArea) {
        bestArea = areas[i];
        bestIndex = i;
        anyValid = true;
      }
    }

    activeCameraIndex = anyValid ? bestIndex : 0;
  }

  // ==================== TARGETING DATA GETTERS ====================
  // These return data from the best (active) camera for use by AlignToAprilTag
  // and heading-lock commands.

  /** @return The name of the camera currently providing the best target. */
  public String getActiveCameraName() {
    return VisionConstants.LIMELIGHT_NAMES[activeCameraIndex];
  }

  /** @return True if a valid AprilTag target is detected on either camera. */
  public boolean hasTarget() {
    for (int i = 0; i < tvEntries.length; i++) {
      if (tvEntries[i].getDouble(0) == 1.0 && ((int) tidEntries[i].getDouble(0)) != 0) {
        return true;
      }
    }
    return false;
  }

  /** @return Horizontal offset from crosshair to target (degrees) from the active camera. */
  public double getTx() {
    return txEntries[activeCameraIndex].getDouble(0);
  }

  /** @return Vertical offset from crosshair to target (degrees) from the active camera. */
  public double getTy() {
    return tyEntries[activeCameraIndex].getDouble(0);
  }

  /** @return Target area as percentage of image (0-100) from the active camera. */
  public double getTa() {
    return taEntries[activeCameraIndex].getDouble(0);
  }

  /** @return The AprilTag ID being tracked by the active camera (0 if none). */
  public int getTid() {
    return (int) tidEntries[activeCameraIndex].getDouble(0);
  }
}
