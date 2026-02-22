package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Distance-based lookup tables for shooter RPM and pivot angle.
 *
 * Uses WPILib's InterpolatingTreeMap for smooth linear interpolation between
 * empirically-measured data points.
 *
 * Tuning guide: Drive to each distance, manually adjust RPM and angle until
 * fuel
 * consistently scores, then record the values here. The interpolation handles
 * all
 * intermediate distances automatically.
 *
 * Hub opening front edge is 72in (1.8288m) off the carpet.
 * Shooter release height is 19.5in (0.4953m) off the carpet.
 * The differential height is ~52.5in (1.3335m).
 * Pivot range: 60deg to 80deg from horizontal.
 */
public final class ShooterInterpolationTable {

    private ShooterInterpolationTable() {
    } // Static utility class

    // ==================== RPM TABLE ====================
    // Maps distance (meters) -> flywheel RPM
    private static final InterpolatingTreeMap<Double, Double> rpmTable = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        // -------------------------------------------------------
        // PLACEHOLDER VALUES - must be tuned on the robot!
        // Distance (m) -> RPM
        // -------------------------------------------------------
        // Close range: low RPM, steep angle (fuel doesn't need much speed)
        rpmTable.put(1.5, 1800.0);
        rpmTable.put(2.0, 2000.0);
        rpmTable.put(2.5, 2200.0);
        rpmTable.put(3.0, 2400.0);
        rpmTable.put(3.5, 2700.0);
        rpmTable.put(4.0, 3000.0);
        rpmTable.put(4.5, 3300.0);
        rpmTable.put(5.0, 3600.0);
        rpmTable.put(5.5, 3900.0);
        rpmTable.put(6.0, 4200.0);
    }

    // ==================== ANGLE TABLE ====================
    // Maps distance (meters) -> pivot angle (degrees from horizontal)
    private static final InterpolatingTreeMap<Double, Double> angleTable = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        // -------------------------------------------------------
        // PLACEHOLDER VALUES - must be tuned on the robot!
        // Distance (m) -> Pivot angle (degrees)
        // At close range, need steeper angle (closer to 80deg) to lob upward
        // At far range, need shallower angle (closer to 60deg) for flatter trajectory
        // -------------------------------------------------------
        angleTable.put(1.5, 78.0);
        angleTable.put(2.0, 76.0);
        angleTable.put(2.5, 74.0);
        angleTable.put(3.0, 72.0);
        angleTable.put(3.5, 70.0);
        angleTable.put(4.0, 68.0);
        angleTable.put(4.5, 66.0);
        angleTable.put(5.0, 64.0);
        angleTable.put(5.5, 62.0);
        angleTable.put(6.0, 60.0);
    }

    /**
     * Get the interpolated flywheel RPM for a given distance.
     *
     * @param distanceMeters horizontal distance to hub in meters
     * @return target flywheel RPM
     */
    public static double getRPM(double distanceMeters) {
        return rpmTable.get(distanceMeters);
    }

    /**
     * Get the interpolated pivot angle for a given distance.
     *
     * @param distanceMeters horizontal distance to hub in meters
     * @return target pivot angle in degrees from horizontal
     */
    public static double getAngleDegrees(double distanceMeters) {
        return angleTable.get(distanceMeters);
    }
}
