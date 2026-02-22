package frc.robot.lib;

public class ShooterSetpoint {

    private final double flywheelRPM;
    private final double pivotAngleDegrees;
    private final boolean isValid;

    public ShooterSetpoint(double flywheelRPM, double pivotAngleDegrees, boolean isValid) {
        this.flywheelRPM = flywheelRPM;
        this.pivotAngleDegrees = pivotAngleDegrees;
        this.isValid = isValid;
    }

    public ShooterSetpoint(double flywheelRPM, double pivotAngleDegrees) {
        this(flywheelRPM, pivotAngleDegrees, true);
    }

    public static ShooterSetpoint fromDistance(double distanceMeters) {
        double rpm = ShooterInterpolationTable.getRPM(distanceMeters);
        double angle = ShooterInterpolationTable.getAngleDegrees(distanceMeters);

        boolean valid = angle >= frc.robot.constants.ShooterPivotConstants.MIN_ANGLE_DEGREES
                && angle <= frc.robot.constants.ShooterPivotConstants.MAX_ANGLE_DEGREES;

        angle = Math.max(
                frc.robot.constants.ShooterPivotConstants.MIN_ANGLE_DEGREES,
                Math.min(angle, frc.robot.constants.ShooterPivotConstants.MAX_ANGLE_DEGREES));

        return new ShooterSetpoint(rpm, angle, valid);
    }

    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public double getPivotAngleDegrees() {
        return pivotAngleDegrees;
    }

    public boolean isValid() {
        return isValid;
    }

    public static final ShooterSetpoint STOWED = new ShooterSetpoint(0.0,
            frc.robot.constants.ShooterPivotConstants.MIN_ANGLE_DEGREES, true);

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
