// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagMaps;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem inspired by Citrus Circuits (1678) 2025 approach.
 *
 * Processes two Limelight 4 cameras for AprilTag-based robot localization using MegaTag2.
 * Both cameras feed into the CTRE SwerveDrivetrain's built-in Kalman filter pose estimator
 * with distance-scaled standard deviations.
 *
 * Key design decisions (from Citrus Circuits):
 * - Distance-scaled std devs: {@code base × avgTagDist}. Farther tags get trusted less.
 * - Heading completely distrusted (theta stddev = 99999). Heading comes from the Pigeon2 gyro.
 * - LL4 IMU mode 4 (internal + external assist) for 1kHz heading updates during rotation.
 * - Both cameras processed independently every cycle - both contribute to the pose estimator.
 */
public class VisionSubsystem extends SubsystemBase {

  // ==================== CONFIGURATION ====================

  /** Base standard deviations for vision measurements. Scaled by avgTagDist per frame. */
  private static final Matrix<N3, N1> BASE_STD_DEVS = VecBuilder.fill(0.3, 0.3, 99999.0);

  /** Maximum angular velocity (deg/s) above which vision updates are rejected. */
  private static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC =
      VisionConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC;

  /** Field boundary margin for out-of-bounds rejection (meters). */
  private static final double FIELD_MARGIN = VisionConstants.FIELD_BORDER_MARGIN;
  private static final double FIELD_LENGTH = VisionConstants.FIELD_LENGTH_METERS;
  private static final double FIELD_WIDTH = VisionConstants.FIELD_WIDTH_METERS;

  /** Pipeline indices */
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
  private int totalAccepted = 0;
  private int totalRejected = 0;

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
    }
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    // --- Update active camera selection (for targeting data getters) ---
    updateActiveCameraSelection();

    // --- Apply alliance-based tag filtering ---
    applyTagFilters();

    // --- Process both cameras for MegaTag2 pose estimation ---
    String[] names = VisionConstants.LIMELIGHT_NAMES;
    Pose2d odoPose = drivetrain.getState().Pose;
    double fusedHeadingDeg = odoPose.getRotation().getDegrees();
    double angularVelocity = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();

    for (int i = 0; i < names.length; i++) {
      processCamera(names[i], odoPose, fusedHeadingDeg, angularVelocity);
    }

    // --- Log telemetry ---
    Logger.recordOutput("Vision/TotalAccepted", totalAccepted);
    Logger.recordOutput("Vision/TotalRejected", totalRejected);
    Logger.recordOutput("Vision/ActiveCamera", VisionConstants.LIMELIGHT_NAMES[activeCameraIndex]);
    Logger.recordOutput("Vision/FusedHeadingDeg", fusedHeadingDeg);
  }

  // ==================== CORE VISION PIPELINE ====================

  /**
   * Process a single camera's MegaTag2 estimate through the validation chain
   * and feed accepted measurements into the drivetrain's pose estimator.
   *
   * Inspired by Citrus Circuits 1678 VisionIOLimelight.update() with minimal,
   * proven filtering:
   * 1. Send robot orientation to Limelight (required for MegaTag2)
   * 2. Get MegaTag2 pose estimate
   * 3. Gate: null / stale / no tags
   * 4. Gate: angular velocity too high
   * 5. Gate: field bounds check
   * 6. Compute distance-scaled standard deviations
   * 7. Feed into pose estimator
   */
  private void processCamera(
      String cameraName, Pose2d odoPose, double fusedHeadingDeg, double angularVelocity) {

    String logPrefix = "Vision/" + cameraName + "/";

    // Step 1: Send current heading to Limelight (MUST happen before reading MT2)
    LimelightHelpers.SetRobotOrientation(cameraName, fusedHeadingDeg, 0, 0, 0, 0, 0);

    // Step 2: Get MegaTag2 estimate
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    // Gate 1: Null / stale / no tags
    if (mt2 == null || mt2.timestampSeconds == 0 || mt2.tagCount == 0) {
      Logger.recordOutput(logPrefix + "Status", "NO_DATA");
      totalRejected++;
      return;
    }

    // Gate 2: Angular velocity - MegaTag2 degrades when spinning fast
    if (Math.abs(angularVelocity) > MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
      Logger.recordOutput(logPrefix + "Status", "ANG_VEL_REJECT");
      totalRejected++;
      return;
    }

    // Gate 3: Field bounds - reject poses outside the field
    double x = mt2.pose.getX();
    double y = mt2.pose.getY();
    if (x < -FIELD_MARGIN || x > FIELD_LENGTH + FIELD_MARGIN
        || y < -FIELD_MARGIN || y > FIELD_WIDTH + FIELD_MARGIN) {
      Logger.recordOutput(logPrefix + "Status", "OUT_OF_BOUNDS");
      totalRejected++;
      return;
    }

    // Compute distance-scaled standard deviations (Citrus Circuits formula)
    // base_stddev × avgTagDist - farther tags get trusted less
    double avgTagDist = mt2.avgTagDist;
    double scaledXY = BASE_STD_DEVS.get(0, 0) * avgTagDist;
    Matrix<N3, N1> stdDevs = VecBuilder.fill(scaledXY, scaledXY, BASE_STD_DEVS.get(2, 0));

    // Feed into the CTRE pose estimator's Kalman filter
    drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);

    // Telemetry
    totalAccepted++;
    Logger.recordOutput(logPrefix + "Status", "ACCEPTED");
    Logger.recordOutput(logPrefix + "Pose", mt2.pose);
    Logger.recordOutput(logPrefix + "TagCount", mt2.tagCount);
    Logger.recordOutput(logPrefix + "AvgTagDist", avgTagDist);
    Logger.recordOutput(logPrefix + "XYStdDev", scaledXY);
    Logger.recordOutput(logPrefix + "OdoDivergence",
        mt2.pose.getTranslation().getDistance(odoPose.getTranslation()));
  }

  // ==================== LL4 IMU MANAGEMENT ====================

  /**
   * Seed the LL4 internal IMU with the external gyro heading.
   * Call this continuously while the robot is disabled (pre-match).
   *
   * IMU Mode 1 (EXTERNAL_SEED): The LL4's internal IMU offset is calibrated
   * to match the external yaw each frame. MT2 still uses external yaw for botpose.
   */
  public void seedIMU() {
    double heading = drivetrain.getState().Pose.getRotation().getDegrees();
    for (String name : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.SetRobotOrientation(name, heading, 0, 0, 0, 0, 0);
      LimelightHelpers.SetIMUMode(name, 1); // EXTERNAL_SEED
    }
  }

  /**
   * Switch LL4 to internal IMU + external assist mode for match play.
   * Call this in autonomousInit() and teleopInit().
   *
   * IMU Mode 4 (INTERNAL_EXTERNAL_ASSIST): Uses the LL4's 1kHz internal IMU
   * for frame-by-frame heading updates while the robot's Pigeon2 corrects
   * for drift over time. This gives MegaTag2 much better heading data during
   * rotation than sending heading over NetworkTables at 50Hz.
   */
  public void enableIMU() {
    for (String name : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.SetIMUMode(name, 4); // INTERNAL_EXTERNAL_ASSIST
      LimelightHelpers.SetIMUAssistAlpha(name, 0.001); // Gentle drift correction
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

  // ==================== ALLIANCE TAG FILTERING ====================

  /**
   * Apply alliance-based tag ID filtering to both cameras.
   * Only processes tags on the robot's own side of the field.
   */
  private void applyTagFilters() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      int[] validTags = alliance.get() == Alliance.Red
          ? AprilTagMaps.RED_SIDE_TAGS
          : AprilTagMaps.BLUE_SIDE_TAGS;
      for (String name : VisionConstants.LIMELIGHT_NAMES) {
        LimelightHelpers.SetFiducialIDFiltersOverride(name, validTags);
      }
    }
  }

  // ==================== ACTIVE CAMERA SELECTION ====================

  /**
   * Select the camera with the best (largest area) target.
   * Used by targeting getters (getTid, getTx, etc.) for alignment commands.
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

    // Default to first camera if none have valid targets
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
