package frc.robot.lib;

/**
 * Represents a complete shooter setpoint: flywheel RPM and pivot angle.
 *
 * <p>The setpoint is computed from the distance to the hub using interpolation tables.
 *
 * @see ShooterInterpolationTable
 */
public class ShooterSetpoint {

  private final double flywheelRPM;
  private final double pivotAngleDegrees;
  private final boolean isValid;

  /**
   * Create a new ShooterSetpoint.
   *
   * @param flywheelRPM target flywheel RPM
   * @param pivotAngleDegrees target pivot angle in degrees
   * @param isValid whether this setpoint is achievable by the hardware
   */
  public ShooterSetpoint(double flywheelRPM, double pivotAngleDegrees, boolean isValid) {
    this.flywheelRPM = flywheelRPM;
    this.pivotAngleDegrees = pivotAngleDegrees;
    this.isValid = isValid;
  }

  public ShooterSetpoint(double flywheelRPM, double pivotAngleDegrees) {
    this(flywheelRPM, pivotAngleDegrees, true);
  }

  /**
   * Compute a ShooterSetpoint from the distance to the hub.
   *
   * <p>Queries the interpolation tables for RPM and pivot angle, then validates that the resulting
   * angle is within the pivot's range of motion.
   *
   * @param distanceMeters 2D distance from robot to hub center (meters)
   * @return a ShooterSetpoint with the appropriate RPM and angle
   */
  public static ShooterSetpoint fromDistance(double distanceMeters) {
    double rpm = ShooterInterpolationTable.getRPM(distanceMeters);
    double angle = ShooterInterpolationTable.getAngleDegrees(distanceMeters);

    // Validate that the angle is within the pivot's physical range
    boolean valid = angle >= frc.robot.constants.ShooterPivotConstants.MIN_ANGLE_DEGREES
        && angle <= frc.robot.constants.ShooterPivotConstants.MAX_ANGLE_DEGREES;

    // Clamp to safe range even if invalid (so motor doesn't go crazy)
    angle = Math.max(
        frc.robot.constants.ShooterPivotConstants.MIN_ANGLE_DEGREES,
        Math.min(angle, frc.robot.constants.ShooterPivotConstants.MAX_ANGLE_DEGREES));

    return new ShooterSetpoint(rpm, angle, valid);
  }

  /** @return the target flywheel RPM */
  public double getFlywheelRPM() {
    return flywheelRPM;
  }

  /** @return the target pivot angle in degrees */
  public double getPivotAngleDegrees() {
    return pivotAngleDegrees;
  }

  /**
   * Whether this setpoint is valid (achievable by the hardware). If the setpoint exceeds hardware
   * limits, it's marked invalid and the robot should NOT fire.
   */
  public boolean isValid() {
    return isValid;
  }

  /** A "stowed" setpoint - pivot at minimum angle, no flywheel spin. */
  public static final ShooterSetpoint STOWED =
      new ShooterSetpoint(0.0, frc.robot.constants.ShooterPivotConstants.MIN_ANGLE_DEGREES, true);

  /**
   * A fixed "fender shot" setpoint for shooting while pressed up against the hub. These values
   * bypass distance-based interpolation for a known reliable shot.
   */
  public static final ShooterSetpoint FENDER_SHOT = new ShooterSetpoint(
      frc.robot.constants.ShooterConstants.FENDER_SHOT_RPM,
      frc.robot.constants.ShooterConstants.FENDER_SHOT_PIVOT_DEGREES,
      true);

  @Override
  public String toString() {
    return String.format(
        "ShooterSetpoint[RPM=%.0f, Angle=%.1fdeg, valid=%b]",
        flywheelRPM, pivotAngleDegrees, isValid);
  }
}
