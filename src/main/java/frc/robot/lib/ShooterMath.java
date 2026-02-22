package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.GameConstants;
import java.util.function.Supplier;

public final class ShooterMath {

    private ShooterMath() {
    }

    public static double getDistanceToHub(Pose2d robotPose) {
        Translation2d hubPosition = getHubPosition();
        return robotPose.getTranslation().getDistance(hubPosition);
    }

    public static Translation2d getHubPosition() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        return isRed ? GameConstants.RED_HUB_CENTER : GameConstants.BLUE_HUB_CENTER;
    }

    public static double getHeadingToHub(Pose2d robotPose) {
        Translation2d hubPosition = getHubPosition();
        double dx = hubPosition.getX() - robotPose.getX();
        double dy = hubPosition.getY() - robotPose.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public static Supplier<ShooterSetpoint> createSetpointSupplier(Supplier<Pose2d> poseSupplier) {
        return new MemoizedSetpointSupplier(poseSupplier);
    }

    private static class MemoizedSetpointSupplier implements Supplier<ShooterSetpoint> {
        private final Supplier<Pose2d> poseSupplier;
        private ShooterSetpoint cached = ShooterSetpoint.STOWED;
        private double lastX = Double.NaN;
        private double lastY = Double.NaN;
        private double lastTheta = Double.NaN;

        MemoizedSetpointSupplier(Supplier<Pose2d> poseSupplier) {
            this.poseSupplier = poseSupplier;
        }

        @Override
        public ShooterSetpoint get() {
            Pose2d pose = poseSupplier.get();
            double x = pose.getX();
            double y = pose.getY();
            double theta = pose.getRotation().getRadians();

            if (x != lastX || y != lastY || theta != lastTheta) {
                lastX = x;
                lastY = y;
                lastTheta = theta;
                double distance = getDistanceToHub(pose);
                cached = ShooterSetpoint.fromDistance(distance);
            }
            return cached;
        }
    }
}
