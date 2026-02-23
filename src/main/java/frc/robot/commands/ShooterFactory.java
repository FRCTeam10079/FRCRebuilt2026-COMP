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

/**
 * Static factory for distance-based shooting commands.
 *
 * <p>All shooting logic funnels through this class so the conditions for firing (flywheel RPM,
 * pivot angle, heading alignment, setpoint validity) are checked in exactly one place.
 *
 * <p>The core shooting sequence is: 1. A Supplier<ShooterSetpoint> provides the current
 * distance-based setpoint (RPM + angle) 2. The flywheel and pivot track that setpoint continuously
 * 3. isOnTarget() gates feeding until everything is ready
 */
public final class ShooterFactory {

  private ShooterFactory() {} // static utility

  // ==================== ON-TARGET GATING ====================

  /**
   * Check whether all conditions for a safe shot are met.
   *
   * <p>Conditions: - Setpoint is valid (non-null, reasonable distance) - Flywheel RPM within
   * ON_TARGET_RPM_PERCENT of target - Pivot angle within SHOOTING_TOLERANCE_DEGREES deg of target -
   * Drivetrain heading within tolerance (checked via headingOnTarget supplier)
   *
   * @param setpointSupplier current setpoint
   * @param shooter flywheel subsystem
   * @param shooterPivot pivot subsystem
   * @param headingOnTarget supplier returning true when the drivetrain heading is aligned
   * @return true when it is safe to feed the ball
   */
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

    // Telemetry for debugging
    SmartDashboard.putBoolean("Shooter/OnTarget/Flywheel", flywheelReady);
    SmartDashboard.putBoolean("Shooter/OnTarget/Pivot", pivotReady);
    SmartDashboard.putBoolean("Shooter/OnTarget/Heading", headingReady);
    SmartDashboard.putBoolean("Shooter/OnTarget/All", flywheelReady && pivotReady && headingReady);

    return flywheelReady && pivotReady && headingReady;
  }

  // ==================== COMMAND FACTORIES ====================

  /**
   * Spin up the flywheel and move the pivot to track a dynamic setpoint. Does NOT feed - just
   * prepares the shooter. Useful for pre-spinning while driving toward a scoring position.
   *
   * @param setpointSupplier dynamic setpoint from ShooterMath
   * @param shooter flywheel subsystem
   * @param shooterPivot pivot subsystem
   * @return command that runs until cancelled
   */
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

  /**
   * Full distance-based shooting sequence (teleop).
   *
   * <p>Simultaneously: - Spins up flywheel to setpoint RPM - Tracks pivot angle from setpoint -
   * Waits until isOnTarget() is true, then feeds the indexer
   *
   * <p>The driver holds this command while aiming the drivetrain at the hub (via heading lock or
   * manual rotation). When all conditions align, the ball is automatically fed.
   *
   * @param setpointSupplier dynamic setpoint
   * @param shooter flywheel subsystem
   * @param shooterPivot pivot subsystem
   * @param indexer indexer subsystem for feeding
   * @param headingOnTarget supplier for heading alignment check
   * @return command that fires when ready, runs until released
   */
  public static Command shoot(
      Supplier<ShooterSetpoint> setpointSupplier,
      ShooterSubsystem shooter,
      ShooterPivotSubsystem shooterPivot,
      IndexerSubsystem indexer,
      Supplier<Boolean> headingOnTarget) {

    return aimAndSpinUp(setpointSupplier, shooter, shooterPivot)
        .alongWith(Commands.waitUntil(
                () -> isOnTarget(setpointSupplier, shooter, shooterPivot, headingOnTarget))
            .andThen(indexer.feedCommand()))
        .withName("ShooterFactory Shoot");
  }

  /**
   * Fender shot - fixed RPM and angle, no distance calculation needed. Good for scoring from right
   * in front of the hub.
   *
   * @param shooter flywheel subsystem
   * @param shooterPivot pivot subsystem
   * @param indexer indexer subsystem
   * @return command that fires a fixed fender shot
   */
  public static Command fenderShot(
      ShooterSubsystem shooter, ShooterPivotSubsystem shooterPivot, IndexerSubsystem indexer) {

    Supplier<ShooterSetpoint> fixed = () -> ShooterSetpoint.FENDER_SHOT;

    return shooter
        .holdRPMCommand(ShooterConstants.FENDER_SHOT_RPM)
        .alongWith(
            shooterPivot.goToAngleCommand(ShooterConstants.FENDER_SHOT_PIVOT_DEGREES),
            Commands.waitUntil(() -> shooter.isReady() && shooterPivot.isAtTarget())
                .andThen(indexer.feedCommand()))
        .withName("ShooterFactory Fender Shot");
  }

  /**
   * Autonomous distance-based shot with timeout.
   *
   * <p>Same as shoot() but with a timeout for auto routines. The heading check is always true in
   * auto (the robot should already be aimed by the trajectory).
   *
   * @param setpointSupplier dynamic setpoint
   * @param shooter flywheel subsystem
   * @param shooterPivot pivot subsystem
   * @param indexer indexer subsystem
   * @return command that shoots with timeout
   */
  public static Command autoShoot(
      Supplier<ShooterSetpoint> setpointSupplier,
      ShooterSubsystem shooter,
      ShooterPivotSubsystem shooterPivot,
      IndexerSubsystem indexer) {

    // In auto, the trajectory handles heading, so heading is always "on target"
    return shoot(setpointSupplier, shooter, shooterPivot, indexer, () -> true)
        .withTimeout(ShooterConstants.AUTO_SHOOT_TIMEOUT)
        .withName("ShooterFactory AutoShoot");
  }

  /**
   * Create a heading-locked drive command that aims the drivetrain at the hub while the driver
   * controls translation.
   *
   * @param drivetrain swerve drivetrain
   * @param xInput driver X (forward) input
   * @param yInput driver Y (strafe) input
   * @param headingSupplier supplier of the target heading to the hub (degrees)
   * @param maxVelocity max translation speed (m/s)
   * @param maxAngularVelocity max angular speed (rad/s)
   * @return command that applies heading lock toward the hub
   */
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
