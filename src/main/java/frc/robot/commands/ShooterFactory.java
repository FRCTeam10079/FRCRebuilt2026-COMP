// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class ShooterFactory {

    private ShooterFactory() {
    }

    public static boolean isOnTarget(
            Supplier<ShooterSetpoint> setpointSupplier,
            ShooterSubsystem shooter,
            ShooterPivotSubsystem shooterPivot,
            Supplier<Boolean> headingOnTarget) {

        ShooterSetpoint sp = setpointSupplier.get();
        if (sp == null || !sp.isValid()) {
            return false;
        }

        boolean flywheelReady = shooter.isAtRPM(sp.getFlywheelRPM());
        boolean pivotReady = shooterPivot.isAtAngle(
                sp.getPivotAngleDegrees(), ShooterPivotConstants.SHOOTING_TOLERANCE_DEGREES);
        boolean headingReady = headingOnTarget.get();

        SmartDashboard.putBoolean("Shooter/OnTarget/Flywheel", flywheelReady);
        SmartDashboard.putBoolean("Shooter/OnTarget/Pivot", pivotReady);
        SmartDashboard.putBoolean("Shooter/OnTarget/Heading", headingReady);
        SmartDashboard.putBoolean("Shooter/OnTarget/All", flywheelReady && pivotReady && headingReady);

        return flywheelReady && pivotReady && headingReady;
    }

    public static Command aimAndSpinUp(
            Supplier<ShooterSetpoint> setpointSupplier,
            ShooterSubsystem shooter,
            ShooterPivotSubsystem shooterPivot) {

        return shooter
                .holdRPMCommand(() -> {
                    ShooterSetpoint sp = setpointSupplier.get();
                    return (sp != null && sp.isValid()) ? sp.getFlywheelRPM() : 0.0;
                })
                .alongWith(shooterPivot.trackAngleCommand(() -> {
                    ShooterSetpoint sp = setpointSupplier.get();
                    return (sp != null && sp.isValid())
                            ? sp.getPivotAngleDegrees()
                            : ShooterPivotConstants.MIN_ANGLE_DEGREES;
                }))
                .withName("ShooterFactory AimAndSpinUp");
    }

    public static Command shoot(
            Supplier<ShooterSetpoint> setpointSupplier,
            ShooterSubsystem shooter,
            ShooterPivotSubsystem shooterPivot,
            IndexerSubsystem indexer,
            Supplier<Boolean> headingOnTarget) {

        return aimAndSpinUp(setpointSupplier, shooter, shooterPivot)
                .alongWith(
                        Commands.waitUntil(
                                () -> isOnTarget(setpointSupplier, shooter, shooterPivot, headingOnTarget))
                                .andThen(indexer.feedCommand()))
                .withName("ShooterFactory Shoot");
    }

    public static Command fenderShot(
            ShooterSubsystem shooter,
            ShooterPivotSubsystem shooterPivot,
            IndexerSubsystem indexer) {

        Supplier<ShooterSetpoint> fixed = () -> ShooterSetpoint.FENDER_SHOT;

        return shooter
                .holdRPMCommand(ShooterConstants.FENDER_SHOT_RPM)
                .alongWith(
                        shooterPivot.goToAngleCommand(ShooterConstants.FENDER_SHOT_PIVOT_DEGREES),
                        Commands.waitUntil(() -> shooter.isReady() && shooterPivot.isAtTarget())
                                .andThen(indexer.feedCommand()))
                .withName("ShooterFactory Fender Shot");
    }

    public static Command autoShoot(
            Supplier<ShooterSetpoint> setpointSupplier,
            ShooterSubsystem shooter,
            ShooterPivotSubsystem shooterPivot,
            IndexerSubsystem indexer) {

        return shoot(setpointSupplier, shooter, shooterPivot, indexer, () -> true)
                .withTimeout(ShooterConstants.AUTO_SHOOT_TIMEOUT)
                .withName("ShooterFactory AutoShoot");
    }

    public static Command aimAtHub(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xInput,
            DoubleSupplier yInput,
            DoubleSupplier headingSupplier,
            double maxVelocity,
            double maxAngularVelocity) {

        return drivetrain
                .headingLockedDriveCommand(xInput, yInput, headingSupplier, maxVelocity, maxAngularVelocity)
                .withName("ShooterFactory AimAtHub");
    }
}
