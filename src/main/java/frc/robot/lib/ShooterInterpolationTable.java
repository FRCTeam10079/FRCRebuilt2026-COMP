package frc.robot.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import java.util.NavigableSet;
import java.util.TreeSet;

/**
 * Distance-based lookup tables for shooter RPM and pivot angle.
 *
 * <p>Uses WPILib's InterpolatingTreeMap for smooth linear interpolation between
 * empirically-measured data points.
 *
 * <p>Tuning guide: Drive to each distance, manually adjust RPM and angle until fuel consistently
 * scores, then record the values here. The interpolation handles all intermediate distances
 * automatically.
 *
 * <p>Hub opening front edge is 72in (1.8288m) off the carpet. Shooter release height is 19.5in
 * (0.4953m) off the carpet. The differential height is ~52.5in (1.3335m). Pivot range: 60deg to
 * 80deg from horizontal.
 */
public final class ShooterInterpolationTable {

  private ShooterInterpolationTable() {} // Static utility class

  // ==================== RPM TABLE ====================
  // Maps distance (meters) -> flywheel RPM
  private static InterpolatingTreeMap<Double, Double> rpmTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final NavigableSet<Double> rpmKeys = new TreeSet<>();

  static {
    // -------------------------------------------------------
    // PLACEHOLDER VALUES - must be tuned on the robot!
    // Distance (m) -> RPM
    // -------------------------------------------------------
    // Close range: low RPM, steep angle (fuel doesn't need much speed)
    putRpm(1.0, 2000.0);
    putRpm(2.2, 2050.0);
    // rpmTable.put(2.5, 2100.0);
    rpmTable.put(3.0, 2355.0);
    // rpmTable.put(3.5, 2500.0);
    rpmTable.put(4.5, 2625.0);
    // rpmTable.put(4.5, 3300.0);
    // rpmTable.put(5.0, 2500.0);
    // rpmTable.put(5.5, 3900.0);
    // rpmTable.put(6.0, 4200.0);
  }

  // ==================== ANGLE TABLE ====================
  // Maps distance (meters) -> pivot angle (degrees from horizontal)
  private static InterpolatingTreeMap<Double, Double> angleTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final NavigableSet<Double> angleKeys = new TreeSet<>();

  static {
    // -------------------------------------------------------
    // PLACEHOLDER VALUES - must be tuned on the robot!
    // Distance (m) -> Pivot angle (degrees)
    // At close range, need steeper angle (closer to 80deg) to lob upward
    // At far range, need shallower angle (closer to 60deg) for flatter trajectory
    // -------------------------------------------------------
    putAngle(1.0, 60.0);
    putAngle(2.0, 60.0);
    // angleTable.put(2.5, 60.0);
    putAngle(3.2, 64.0);
    // angleTable.put(3.5, 64.0);
    putAngle(4.5, 72.5);
    // Don't need 5, max is 4.5
  }

  /**
   * Get the interpolated flywheel RPM for a given distance.
   *
   * @param distance horizontal distance to hub in meters
   * @return target flywheel RPM
   */
  public static AngularVelocity getRPM(Distance distance) {
    return RPM.of(rpmTable.get(distance.in(Meters)));
  }

  // ==================== TIME OF FLIGHT TABLE ====================
  // Maps distance (meters) -> time of flight (seconds)
  // Time from ball leaving the shooter to arriving at the hub.
  // CRITICAL for shoot-on-the-move - determines lookahead offset.
  private static InterpolatingTreeMap<Double, Double> tofTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  static {
    // -------------------------------------------------------
    // TODO: MUST BE MEASURED ON REAL ROBOT!
    // Use slow-motion video (240fps phone camera) from the side.
    // Time from ball exit to hub entry at each distance.
    // These are ESTIMATED placeholders based on physics:
    // - Hub height delta ~1.33m, pivot angles 60-80deg
    // - At 60deg launch, v~15m/s, horizontal component ~7.5m/s
    // - TOF ~ distance / horizontal_velocity (rough)
    // -------------------------------------------------------
    tofTable.put(1.0, 0.25); // very close - short flight
    tofTable.put(1.5, 0.30); // TODO: TUNE - placeholder estimate
    tofTable.put(2.0, 0.35); // TODO: TUNE - placeholder estimate
    tofTable.put(2.5, 0.40); // TODO: TUNE - placeholder estimate
    tofTable.put(3.0, 0.45); // TODO: TUNE - placeholder estimate
    tofTable.put(3.5, 0.50); // TODO: TUNE - placeholder estimate
    tofTable.put(4.0, 0.55); // TODO: TUNE - placeholder estimate
    tofTable.put(4.5, 0.60); // TODO: TUNE - placeholder estimate
    tofTable.put(5.0, 0.65); // TODO: TUNE - placeholder estimate
  }

  /**
   * Get the interpolated pivot angle for a given distance.
   *
   * @param distance horizontal distance to hub in meters
   * @return target pivot angle in degrees from horizontal
   */
  public static Angle getAngle(Distance distance) {
    return Degrees.of(angleTable.get(distance.in(Meters)));
  }

  /**
   * Get the interpolated time-of-flight for a given distance.
   *
   * <p>This is used by the LaunchCalculator for velocity-compensated lookahead. The ball's flight
   * time determines how much the robot's velocity offsets the effective aim point.
   *
   * @param distanceMeters horizontal distance to hub in meters
   * @return estimated time of flight in seconds
   */
  public static double getTimeOfFlight(double distanceMeters) {
    return tofTable.get(distanceMeters);
  }

  public static void hotSwapTofValues(Double key, Double newValue) {
    tofTable.put(key, newValue);

    System.out.println(tofTable.get(key));
  }

  public static void hotSwapRPMValues(Double key, Double newValue) {
    putRpm(key, newValue);

    System.out.println(rpmTable.get(key));
  }

  public static void hotSwapAngleValues(Double key, Double newValue) {
    putAngle(key, newValue);

    System.out.println(angleTable.get(key));
  }

  public static double getClosestRPMKey(double distanceMeters) {
    return getClosestKey(rpmKeys, distanceMeters);
  }

  public static double getClosestAngleKey(double distanceMeters) {
    return getClosestKey(angleKeys, distanceMeters);
  }

  private static double getClosestKey(NavigableSet<Double> keys, double queryMeters) {
    if (keys.isEmpty()) {
      return queryMeters;
    }

    Double lower = keys.floor(queryMeters);
    Double upper = keys.ceiling(queryMeters);

    if (lower == null) return upper;
    if (upper == null) return lower;
    return (queryMeters - lower) <= (upper - queryMeters) ? lower : upper;
  }

  private static void putRpm(Double key, Double value) {
    rpmTable.put(key, value);
    rpmKeys.add(key);
  }

  private static void putAngle(Double key, Double value) {
    angleTable.put(key, value);
    angleKeys.add(key);
  }
}
