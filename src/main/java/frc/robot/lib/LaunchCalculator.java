package frc.robot.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;

/**
 * Core shoot-on-the-move calculator, adapted from Mechanical Advantage (6328).
 *
 * <p>This class computes velocity-compensated launch parameters so the robot can shoot while
 * moving. Instead of aiming where the hub IS, it computes where to aim based on where the robot's
 * velocity would carry the ball during its flight time.
 *
 * <p>Algorithm overview: 1. Project the robot pose forward by a phase delay (latency compensation)
 * 2. Iteratively solve for a self-consistent (distance, TOF) pair using the robot's field-relative
 * velocity - the "lookahead" 3. Look up RPM, pivot angle, and heading from the lookahead distance
 * 4. Compute angular velocity feedforward for smooth heading and pivot tracking
 *
 * <p>Usage: Call {@link #update(Pose2d, ChassisSpeeds, Rotation2d)} once per robot loop, then read
 * the latest {@link LaunchParameters} via {@link #getParameters()}.
 */
public class LaunchCalculator {

  // ==================== SINGLETON ====================
  private static LaunchCalculator instance;

  public static LaunchCalculator getInstance() {
    if (instance == null) {
      instance = new LaunchCalculator();
    }
    return instance;
  }

  // ==================== TUNING CONSTANTS ====================

  /**
   * Phase delay in seconds - compensates for latency between sensing and actuation. Projection
   * forward along current velocity by this amount.
   *
   * <p>TODO: TUNE ON THE ROBOT - start at 0.03 (30ms) and adjust if shots consistently lead/lag.
   */
  private static final double PHASE_DELAY_SECONDS = 0.03;

  /**
   * Number of iterations for the lookahead convergence loop. 20 is more than enough for convergence
   * - MA uses 20.
   */
  private static final int LOOKAHEAD_ITERATIONS = 20;

  /** Robot loop period in seconds (50Hz = 0.02s). */
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  /**
   * Moving average filter window for pivot angle velocity feedforward. Smooths out the derivative
   * to prevent jitter.
   *
   * <p>0.4s window = 20 samples at 50Hz (same as MA).
   */
  private static final double PIVOT_ANGLE_FILTER_WINDOW_SECONDS = 0.4;

  /**
   * Moving average filter window for drive heading velocity feedforward. Wider window = smoother
   * but more lag.
   *
   * <p>1.5s window = 75 samples at 50Hz (same as MA).
   */
  private static final double DRIVE_ANGLE_FILTER_WINDOW_SECONDS = 1.5;

  /**
   * Minimum distance (meters) at which shoot-on-the-move is valid. Below this the robot is too
   * close for the system to work well.
   *
   * <p>TODO: TUNE ON THE ROBOT - should match the closest distance in the interp tables.
   */
  private static final double MIN_VALID_DISTANCE = 0.8;

  /**
   * Maximum distance (meters) at which shoot-on-the-move is valid.
   *
   * <p>TODO: TUNE ON THE ROBOT - should match the farthest distance in the interp tables.
   */
  private static final double MAX_VALID_DISTANCE = 5.0;

  /**
   * Shooter offset from robot center as (x, y) in robot frame.
   *
   * <p>x = forward/back from center (positive = forward) y = left/right from center (positive =
   * left)
   *
   * <p>TODO: MEASURE ON THE ROBOT - set to (0, 0) if shooter is roughly centered. If lateral offset
   * is > ~5cm, measure and set it for accurate heading compensation.
   */
  private static final double SHOOTER_OFFSET_X = 0.0; // meters, placeholder

  private static final double SHOOTER_OFFSET_Y = 0.0; // meters, placeholder

  // ==================== FILTERS ====================

  private final LinearFilter pivotAngleFilter =
      LinearFilter.movingAverage((int) (PIVOT_ANGLE_FILTER_WINDOW_SECONDS / LOOP_PERIOD_SECONDS));

  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (DRIVE_ANGLE_FILTER_WINDOW_SECONDS / LOOP_PERIOD_SECONDS));

  // ==================== STATE ====================

  private double lastPivotAngleDegrees = Double.NaN;
  private Rotation2d lastDriveAngle = null;
  private LaunchParameters latestParameters = null;

  // ==================== DATA RECORD ====================

  /**
   * Complete set of launch parameters computed each cycle.
   *
   * @param isValid whether all conditions are met to shoot from this location/distance
   * @param driveAngle the field-relative heading the robot should face (Rotation2d)
   * @param driveVelocityRadPerSec angular velocity feedforward for heading tracking (rad/s)
   * @param pivotAngleDegrees target pivotangle for the shooter (degrees)
   * @param pivotVelocityDegPerSec angular velocity feedforward for pivot tracking (deg/s)
   * @param flywheelRPM target flywheel RPM from interpolation table
   * @param lookaheadDistance the velocity-compensated effective distance (meters)
   * @param rawDistance the static distance to hub with no velocity compensation (meters)
   * @param timeOfFlight estimated time for the ball to reach the hub (seconds)
   */
  public record LaunchParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocityRadPerSec,
      double pivotAngleDegrees,
      double pivotVelocityDegPerSec,
      double flywheelRPM,
      double lookaheadDistance,
      double rawDistance,
      double timeOfFlight) {}

  // ==================== CORE UPDATE ====================

  /**
   * Compute all launch parameters for the current robot state.
   *
   * <p>Call this ONCE per robot loop (typically from robotPeriodic or the drive command). The
   * result is cached and returned by {@link #getParameters()} until {@link #clearParameters()} is
   * called.
   *
   * @param robotPose current field-relative robot pose from drivetrain odometry
   * @param robotRelativeVelocity current robot-relative ChassisSpeeds from drivetrain
   * @param robotHeading current robot heading (for field-relative velocity conversion)
   */
  public void update(
      Pose2d robotPose, ChassisSpeeds robotRelativeVelocity, Rotation2d robotHeading) {
    if (latestParameters != null) {
      // Already computed this cycle - skip
      return;
    }

    // ---- Step 1: Phase delay compensation ----
    // Project the robot pose forward by the phase delay to compensate for system
    // latency
    Pose2d estimatedPose = robotPose.exp(new Twist2d(
        robotRelativeVelocity.vxMetersPerSecond * PHASE_DELAY_SECONDS,
        robotRelativeVelocity.vyMetersPerSecond * PHASE_DELAY_SECONDS,
        robotRelativeVelocity.omegaRadiansPerSecond * PHASE_DELAY_SECONDS));

    // ---- Step 2: Determine target ----
    Translation2d target = ShooterMath.getHubPosition();

    // Compute shooter position on the field (applying robot-frame offset)
    Translation2d shooterFieldPos = estimatedPose
        .getTranslation()
        .plus(new Translation2d(SHOOTER_OFFSET_X, SHOOTER_OFFSET_Y)
            .rotateBy(estimatedPose.getRotation()));

    double rawDistance = target.getDistance(shooterFieldPos);

    // ---- Step 3: Field-relative velocity ----
    ChassisSpeeds fieldVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, robotHeading);
    double fieldVelX = fieldVelocity.vxMetersPerSecond;
    double fieldVelY = fieldVelocity.vyMetersPerSecond;

    // ---- Step 4: Iterative lookahead convergence loop ----
    // The ball leaves the launcher with the robot's velocity superimposed.
    // We iterate to find a self-consistent (distance, TOF) pair where:
    // - distance determines TOF (from interpolation table)
    // - TOF determines how far the robot's velocity offsets the effective aim point
    // - the new aim point changes the distance, which changes TOF, etc.
    double timeOfFlight = ShooterInterpolationTable.getTimeOfFlight(rawDistance);
    Translation2d lookaheadPos = shooterFieldPos;
    double lookaheadDistance = rawDistance;

    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      timeOfFlight = ShooterInterpolationTable.getTimeOfFlight(lookaheadDistance);
      double offsetX = fieldVelX * timeOfFlight;
      double offsetY = fieldVelY * timeOfFlight;
      lookaheadPos = shooterFieldPos.plus(new Translation2d(offsetX, offsetY));
      lookaheadDistance = target.getDistance(lookaheadPos);
    }

    // ---- Step 5: Compute drive heading angle ----
    // The robot should face from the lookahead position toward the target
    Rotation2d driveAngle = target.minus(lookaheadPos).getAngle();

    // If the shooter has a significant lateral offset, apply asin correction
    // (similar to MA's getDriveAngleWithLauncherOffset)
    if (Math.abs(SHOOTER_OFFSET_Y) > 0.01) {
      double distForOffset = target.getDistance(estimatedPose.getTranslation());
      double offsetAngleRad =
          Math.asin(MathUtil.clamp(SHOOTER_OFFSET_Y / distForOffset, -1.0, 1.0));
      driveAngle = driveAngle.plus(Rotation2d.fromRadians(offsetAngleRad));
      // Rotate 180 deg if the shooter fires backwards (like MA's launcher)
      // Our shooter fires forward, so no rotation needed
    }

    // ---- Step 6: Compute setpoints from lookahead distance ----
    ShooterSetpoint setpoint = ShooterSetpoint.fromDistance(lookaheadDistance);
    double pivotAngleDegrees = setpoint.getPivotAngleDegrees();
    double flywheelRPM = setpoint.getFlywheelRPM();

    // ---- Step 7: Compute angular velocity feedforwards via derivative + filter
    // ----
    double pivotVelocityDegPerSec = 0.0;
    double driveVelocityRadPerSec = 0.0;

    if (!Double.isNaN(lastPivotAngleDegrees) && lastDriveAngle != null) {
      pivotVelocityDegPerSec = pivotAngleFilter.calculate(
          (pivotAngleDegrees - lastPivotAngleDegrees) / LOOP_PERIOD_SECONDS);
      driveVelocityRadPerSec = driveAngleFilter.calculate(
          driveAngle.minus(lastDriveAngle).getRadians() / LOOP_PERIOD_SECONDS);
    } else {
      // First cycle - reset filters
      pivotAngleFilter.reset();
      driveAngleFilter.reset();
    }

    lastPivotAngleDegrees = pivotAngleDegrees;
    lastDriveAngle = driveAngle;

    // ---- Step 8: Validity check ----
    boolean isValid = setpoint.isValid()
        && lookaheadDistance >= MIN_VALID_DISTANCE
        && lookaheadDistance <= MAX_VALID_DISTANCE;

    // ---- Step 9: Build the result ----
    latestParameters = new LaunchParameters(
        isValid,
        driveAngle,
        driveVelocityRadPerSec,
        pivotAngleDegrees,
        pivotVelocityDegPerSec,
        flywheelRPM,
        lookaheadDistance,
        rawDistance,
        timeOfFlight);

    // ---- Step 10: Telemetry ----
    SmartDashboard.putNumber("LaunchCalc/RawDistance", rawDistance);
    SmartDashboard.putNumber("LaunchCalc/LookaheadDistance", lookaheadDistance);
    SmartDashboard.putNumber("LaunchCalc/TimeOfFlight", timeOfFlight);
    SmartDashboard.putNumber("LaunchCalc/DriveAngleDeg", driveAngle.getDegrees());
    SmartDashboard.putNumber("LaunchCalc/DriveVelocityRadPerSec", driveVelocityRadPerSec);
    SmartDashboard.putNumber("LaunchCalc/PivotAngleDeg", pivotAngleDegrees);
    SmartDashboard.putNumber("LaunchCalc/PivotVelocityDegPerSec", pivotVelocityDegPerSec);
    SmartDashboard.putNumber("LaunchCalc/FlywheelRPM", flywheelRPM);
    SmartDashboard.putBoolean("LaunchCalc/IsValid", isValid);
    SmartDashboard.putNumber("LaunchCalc/FieldVelX", fieldVelX);
    SmartDashboard.putNumber("LaunchCalc/FieldVelY", fieldVelY);
  }

  /**
   * Get the latest computed launch parameters.
   *
   * <p>Returns null if {@link #update} has not been called this cycle (i.e., if
   * {@link #clearParameters()} was called and update hasn't been called yet).
   *
   * <p>If null, callers should fall back to static/stop-and-shoot behavior.
   *
   * @return the latest LaunchParameters, or null
   */
  public LaunchParameters getParameters() {
    return latestParameters;
  }

  /**
   * Clear the cached parameters. Call this at the START of each robot periodic loop so that
   * parameters are recomputed fresh each cycle.
   */
  public void clearParameters() {
    latestParameters = null;
  }

  /**
   * Get the raw (non-velocity-compensated) time of flight for a given distance. Useful for velocity
   * limiting calculations.
   *
   * @param distanceMeters distance to hub in meters
   * @return estimated time of flight in seconds
   */
  public double getNaiveTOF(double distanceMeters) {
    return ShooterInterpolationTable.getTimeOfFlight(distanceMeters);
  }

  /**
   * Create a ShooterSetpoint supplier backed by the LaunchCalculator. This returns setpoints
   * computed from the velocity-compensated lookahead distance.
   *
   * <p>When the calculator has no valid parameters (e.g., first cycle), falls back to the static
   * distance-based setpoint from the provided pose supplier.
   *
   * @param poseFallbackSupplier supplier for robot pose (used as fallback when LaunchCalc has no
   *     data)
   * @return a supplier producing velocity-compensated ShooterSetpoints
   */
  public Supplier<ShooterSetpoint> createLaunchSetpointSupplier(
      Supplier<Pose2d> poseFallbackSupplier) {
    return () -> {
      LaunchParameters params = getParameters();
      if (params != null) {
        return new ShooterSetpoint(
            params.flywheelRPM(), params.pivotAngleDegrees(), params.isValid());
      }
      // Fallback: static distance-based setpoint
      double distance = ShooterMath.getDistanceToHub(poseFallbackSupplier.get());
      return ShooterSetpoint.fromDistance(distance);
    };
  }

  /**
   * Reset all internal state (filters, last angles). Call when the robot is disabled or the
   * shooting system is not active, to prevent stale filter data.
   */
  public void reset() {
    pivotAngleFilter.reset();
    driveAngleFilter.reset();
    lastPivotAngleDegrees = Double.NaN;
    lastDriveAngle = null;
    latestParameters = null;
  }
}
