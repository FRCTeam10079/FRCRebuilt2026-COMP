// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Vision subsystem using dual Limelight 4 cameras for AprilTag-based robot
 * localization.
 *
 * Uses MegaTag2 only - no MegaTag1 fallback.
 *
 * Filtering approach:
 * - Full 6-DOF Pigeon2 orientation sent to Limelights every frame via
 * SetRobotOrientation()
 * - 5deg heading divergence gate rejects measurements when vision/gyro disagree
 * - Hard 3m max tag distance cutoff eliminates noisy distant measurements
 * - Cubic std dev for single tags (0.3 * d^3), linear for multi-tag (0.3 * d)
 * - Limelight rotation is never trusted (theta stddev = 9999999)
 * - 50ms transmission delay subtracted from timestamps for accurate Kalman
 * filter fusion
 * - Timestamps computed as FPGA_now - captureLatency - pipelineLatency -
 * transmissionDelay (bypasses unreliable NT server timestamps)
 *
 * Both cameras feed into the CTRE SwerveDrivetrain's built-in Kalman filter
 * pose estimator.
 *
 * Dashboard controls:
 * - "Vision Enabled" - toggle vision processing on/off at runtime
 * - "Unconditionally Trust Vision" - debug mode that force-accepts + resets
 * odometry
 */
public class VisionSubsystem extends SubsystemBase {

  // ==================== CONFIGURATION ====================

  private static final double FIELD_MARGIN = VisionConstants.FIELD_BORDER_MARGIN;
  private static final double FIELD_LENGTH = VisionConstants.FIELD_LENGTH_METERS;
  private static final double FIELD_WIDTH = VisionConstants.FIELD_WIDTH_METERS;

  private static final int PIPELINE_APRILTAG = 0;
  private static final int PIPELINE_DISABLED = 1;

  // ==================== DASHBOARD CONTROLS ====================

  /** Runtime toggle to enable/disable vision pose estimation. */
  public static final LoggedDashboardChooser<Boolean> visionEnabled;

  /**
   * Debug toggle: when ON, force-accepts vision with tiny std devs and resets
   * odometry.
   */
  public static final LoggedDashboardChooser<Boolean> unconditionallyTrustVision;

  static {
    visionEnabled = new LoggedDashboardChooser<>("Vision Enabled");
    visionEnabled.addDefaultOption("on", true);
    visionEnabled.addOption("off", false);

    unconditionallyTrustVision = new LoggedDashboardChooser<>("Unconditionally Trust Vision");
    unconditionallyTrustVision.addDefaultOption("off", false);
    unconditionallyTrustVision.addOption("on", true);
  }

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
  private int totalRejected = 0;

  /**
   * Creates a new VisionSubsystem.
   *
   * @param drivetrain The CommandSwerveDrivetrain to feed vision measurements
   *                   into.
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
    }
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    // Update active camera selection (for targeting data getters)
    updateActiveCameraSelection();

    // Skip pose estimation if vision is disabled via dashboard
    if (!Boolean.TRUE.equals(visionEnabled.get())) {
      Logger.recordOutput("Vision/Enabled", false);
      return;
    }
    Logger.recordOutput("Vision/Enabled", true);

    // Gather Pigeon2 orientation data - all 6 DOF for maximum MT2 accuracy
    String[] names = VisionConstants.LIMELIGHT_NAMES;
    Pose2d odoPose = drivetrain.getState().Pose;
    double yawDeg = odoPose.getRotation().getDegrees();
    double yawRateDeg = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
    double pitchDeg = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    double pitchRateDeg = drivetrain.getPigeon2().getAngularVelocityYWorld().getValueAsDouble();
    double rollDeg = drivetrain.getPigeon2().getRoll().getValueAsDouble();
    double rollRateDeg = drivetrain.getPigeon2().getAngularVelocityXWorld().getValueAsDouble();

    // Process both cameras
    for (int i = 0; i < names.length; i++) {
      processCamera(
          names[i], odoPose, yawDeg, yawRateDeg, pitchDeg, pitchRateDeg, rollDeg, rollRateDeg);
    }

    // Log global telemetry
    Logger.recordOutput("Vision/TotalMT2Accepted", totalMt2Accepted);
    Logger.recordOutput("Vision/TotalRejected", totalRejected);
    Logger.recordOutput("Vision/ActiveCamera", VisionConstants.LIMELIGHT_NAMES[activeCameraIndex]);
    Logger.recordOutput("Vision/FusedHeadingDeg", yawDeg);
    Logger.recordOutput(
        "Vision/Pigeon2RawYawDeg", drivetrain.getPigeon2().getYaw().getValueAsDouble());
  }

  // ==================== CORE VISION PIPELINE ====================

  /**
   * Process a single camera's MegaTag2 estimate through the validation chain and
   * feed accepted
   * measurements into the drivetrain's pose estimator.
   *
   * Pipeline:
   * 1. Send full 6-DOF Pigeon2 orientation to Limelight (required for MT2)
   * 2. Read MT2 estimate
   * 3. Validate through rejection gates
   * 4. Compute distance-scaled std devs and feed to pose estimator
   */
  private void processCamera(
      String cameraName,
      Pose2d odoPose,
      double yawDeg,
      double yawRateDeg,
      double pitchDeg,
      double pitchRateDeg,
      double rollDeg,
      double rollRateDeg) {

    String logPrefix = "Vision/" + cameraName + "/";

    // Step 1: Send full 6-DOF Pigeon2 orientation to Limelight every frame.
    // This is essential for MegaTag2 - it pins the rotation solve to the gyro,
    // leaving only translation to be solved from AprilTag geometry.
    // All 6 values (yaw, yawRate, pitch, pitchRate, roll, rollRate) improve
    // accuracy.
    LimelightHelpers.SetRobotOrientation(
        cameraName, yawDeg, yawRateDeg, pitchDeg, pitchRateDeg, rollDeg, rollRateDeg);

    // Step 2: Read MT2 estimate
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    // Step 3: Compute timestamp using "current time minus all latencies" method.
    // This is more robust than using NT server timestamps (which have ambiguous
    // capture latency handling). We read capture + pipeline latency separately
    // and subtract them plus a fixed network transmission delay from FPGA now.
    double cl = LimelightHelpers.getLatency_Capture(cameraName);
    double tl = LimelightHelpers.getLatency_Pipeline(cameraName);
    double timestamp = Timer.getFPGATimestamp()
        - (cl + tl) / 1000.0
        - VisionConstants.LIMELIGHT_TRANSMISSION_DELAY;

    // Step 4: Validate and accept
    processMT2(mt2, cameraName, odoPose, timestamp, logPrefix);
  }

  /**
   * Validate a MegaTag2 estimate through the rejection chain and feed to
   * drivetrain if accepted.
   *
   * Rejection gates (in order):
   * 1. No target / no tags / null data
   * 2. Empty pose (Limelight returned zeros)
   * 3. Off-field (outside field bounds + 0.15m margin)
   * 4. Heading divergence > 5deg (vision heading vs pose estimator heading)
   * 5. Tags too far (avgDist > 3m)
   *
   * Std dev scaling:
   * - 1 tag: 0.3 * d^3 (cubic - heavily penalizes distant single-tag)
   * - 2+ tags: 0.3 * d (linear - multi-tag geometry is reliable)
   * - Theta: always 9999999 (never trust Limelight rotation)
   */
  private void processMT2(
      LimelightHelpers.PoseEstimate mt2,
      String cameraName,
      Pose2d odoPose,
      double timestamp,
      String logPrefix) {

    // Gate 1: No data - null, stale, or no tags visible
    if (mt2 == null || mt2.timestampSeconds == 0 || mt2.tagCount < 1) {
      Logger.recordOutput(logPrefix + "MT2/Status", "NO_DATA");
      return;
    }

    // Check for "unconditionally trust vision" debug mode
    if (Boolean.TRUE.equals(unconditionallyTrustVision.get())) {
      drivetrain.addVisionMeasurement(
          mt2.pose,
          timestamp,
          VecBuilder.fill(0.01, 0.01, 1));
      drivetrain.resetPose(mt2.pose);
      Logger.recordOutput(logPrefix + "MT2/Status", "UNCONDITIONAL_TRUST");
      Logger.recordOutput(logPrefix + "MT2/Pose", mt2.pose);
      totalMt2Accepted++;
      return;
    }

    // Gate 2: Empty pose - Limelight returned a zeroed-out pose
    if (mt2.pose.equals(new Pose2d())) {
      Logger.recordOutput(logPrefix + "MT2/Status", "EMPTY_POSE");
      totalRejected++;
      return;
    }

    // Gate 3: Field bounds - reject poses outside the field (0.15m margin)
    double x = mt2.pose.getX();
    double y = mt2.pose.getY();
    if (x < -FIELD_MARGIN
        || x > FIELD_LENGTH + FIELD_MARGIN
        || y < -FIELD_MARGIN
        || y > FIELD_WIDTH + FIELD_MARGIN) {
      Logger.recordOutput(logPrefix + "MT2/Status", "OUT_OF_BOUNDS");
      totalRejected++;
      return;
    }

    // Gate 4: Heading divergence - if MT2 heading disagrees with pose estimator
    // heading by more than 5deg, reject. This catches the "bad heading -> bad MT2
    // ->
    // worse heading" death spiral. Since we send our heading to MT2, the returned
    // heading should nearly match - if it doesn't, something is wrong.
    double headingDivergenceDeg = Math.abs(
        odoPose.getRotation().minus(mt2.pose.getRotation()).getDegrees());
    Logger.recordOutput(logPrefix + "MT2/HeadingDivergenceDeg", headingDivergenceDeg);

    if (headingDivergenceDeg > VisionConstants.HEADING_DIVERGENCE_THRESHOLD_DEG) {
      Logger.recordOutput(logPrefix + "MT2/Status", "HEADING_DIVERGE");
      totalRejected++;
      return;
    }

    // Gate 5: Tags too far - distant tags produce noisy measurements
    double avgTagDist = mt2.avgTagDist;
    if (avgTagDist > VisionConstants.MAX_TAG_DISTANCE) {
      Logger.recordOutput(logPrefix + "MT2/Status", "DIST_REJECT");
      totalRejected++;
      return;
    }

    // ===== ACCEPTED - compute distance-scaled standard deviations =====

    double xyStdDev = VisionConstants.MT2_BASE_XY_STDDEV;

    if (mt2.tagCount < 2) {
      // Single tag: cubic scaling - heavily penalizes distant single-tag solves.
      // At 1m: 0.3, at 2m: 2.4, at 3m: 8.1
      xyStdDev *= avgTagDist * avgTagDist * avgTagDist;
    } else {
      // Multi-tag: linear scaling - geometric multi-tag solve is much more reliable.
      // At 1m: 0.3, at 2m: 0.6, at 3m: 0.9
      xyStdDev *= avgTagDist;
    }

    // Feed into the CTRE pose estimator's Kalman filter.
    // Timestamp was computed in processCamera() as:
    // FPGA_now - captureLatency - pipelineLatency - transmissionDelay
    // The drivetrain's addVisionMeasurement converts FPGA -> CTRE time internally.
    drivetrain.addVisionMeasurement(
        mt2.pose,
        timestamp,
        VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.ROTATION_STDDEV));

    // Telemetry
    totalMt2Accepted++;
    Logger.recordOutput(logPrefix + "MT2/Status", "ACCEPTED");
    Logger.recordOutput(logPrefix + "MT2/Pose", mt2.pose);
    Logger.recordOutput(logPrefix + "MT2/TagCount", mt2.tagCount);
    Logger.recordOutput(logPrefix + "MT2/AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "MT2/XYStdDev", xyStdDev);
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
   * Select the camera with the best (largest area) target. Used by targeting
   * getters (getTid,
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

  /**
   * @return Horizontal offset from crosshair to target (degrees) from the active
   *         camera.
   */
  public double getTx() {
    return txEntries[activeCameraIndex].getDouble(0);
  }

  /**
   * @return Vertical offset from crosshair to target (degrees) from the active
   *         camera.
   */
  public double getTy() {
    return tyEntries[activeCameraIndex].getDouble(0);
  }

  /**
   * @return Target area as percentage of image (0-100) from the active camera.
   */
  public double getTa() {
    return taEntries[activeCameraIndex].getDouble(0);
  }

  /** @return The AprilTag ID being tracked by the active camera (0 if none). */
  public int getTid() {
    return (int) tidEntries[activeCameraIndex].getDouble(0);
  }
}
