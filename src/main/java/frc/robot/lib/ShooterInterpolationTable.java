package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public final class ShooterInterpolationTable {

    private ShooterInterpolationTable() {
    }

    private static final InterpolatingTreeMap<Double, Double> rpmTable = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
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

    private static final InterpolatingTreeMap<Double, Double> angleTable = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
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

    public static double getRPM(double distanceMeters) {
        return rpmTable.get(distanceMeters);
    }

    public static double getAngleDegrees(double distanceMeters) {
        return angleTable.get(distanceMeters);
    }
}
