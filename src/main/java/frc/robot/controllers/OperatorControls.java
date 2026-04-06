// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ClimbConstants;
import frc.robot.lib.ShooterInterpolationTable;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.statemachine.FuelState;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Operator controller bindings (Port 1). State-machine management, safety overrides, and shooter
 * pivot manual control. Mechanism actions route through the {@link Superstructure}.
 */
public final class OperatorControls {

  private OperatorControls() {} // Static utility class

  /**
   * Bind all operator controls.
   *
   * @param operator the operator's Xbox controller
   * @param superstructure the Superstructure coordinator
   * @param shooterPivot shooter pivot subsystem (for manual override + homing, bypasses
   *     Superstructure)
   * @param climber climber subsystem (for climb safety interlock)
   * @param stateMachine global robot state machine
   * @param setpointSupplier memoized distance-based setpoint supplier
   * @param hubDistanceSupplier distance to hub supplier (for tuning)
   * @param climbApproachCommandFactory factory that creates a fresh command to pathfind to the
   *     climb approach point
   * @param climbEntryCommandFactory factory that creates a fresh command to pathfind from the
   *     approach point into the climb entry point
   * @param drivetrain swerve drivetrain subsystem (for auto-climb final nudge)
   */
  public static void configure(
      CommandXboxController operator,
      Superstructure superstructure,
      ShooterPivotSubsystem shooterPivot,
      ClimberSubsystem climber,
      RobotStateMachine stateMachine,
      Supplier<ShooterSetpoint> setpointSupplier,
      Supplier<Distance> hubDistanceSupplier,
      Supplier<Command> climbApproachCommandFactory,
      Supplier<Command> climbEntryCommandFactory,
      CommandSwerveDrivetrain drivetrain) {

    // ==================== INVENTORY ====================
    // Y - Human-in-the-loop toggle EMPTY <-> LOADED
    operator
        .y()
        .onTrue(Commands.runOnce(() -> stateMachine.setFuelState(
            stateMachine.getFuelState() == FuelState.LOADED ? FuelState.EMPTY : FuelState.LOADED)));

    // ==================== CLIMB PATHFIND ====================
    // D-Pad Up - Pathfind to selected climb lane (hold to pathfind, release to
    // stop)
    operator
        .povUp()
        .whileTrue(Commands.defer(
            () -> Commands.sequence(
                climbApproachCommandFactory.get(), climbEntryCommandFactory.get()),
            Set.of(drivetrain)));

    // D-Pad Down - Full auto-climb sequence:
    // 1. Pathfind to the approach point in front of climb target
    // 2. While driving final entry path, switch to CLIMB mode and raise climber
    // 3. Retract climber to pull robot up
    // 4. Final small forward nudge (live tunable, default 1 inch)
    operator
        .povDown()
        .onTrue(Commands.defer(
            () -> {
              SwerveRequest.ApplyRobotSpeeds finalForwardRequest =
                  new SwerveRequest.ApplyRobotSpeeds();

              double finalForwardSpeedMps =
                  Math.max(0.05, Math.abs(ClimbConstants.getAutoClimbFinalForwardSpeedMps()));
              double finalForwardDistanceMeters =
                  Math.max(0.0, Math.abs(ClimbConstants.getAutoClimbFinalForwardDistanceMeters()));
              double postMoveSettleSeconds =
                  Math.max(0.0, ClimbConstants.getAutoClimbPostMoveSettleSeconds());
              double finalForwardTimeSec = finalForwardDistanceMeters / finalForwardSpeedMps;

              return Commands.sequence(
                      // Phase 1: Pathfind to climb approach point
                      climbApproachCommandFactory.get(),
                      // Phase 2: Final entry drive + start climb in parallel
                      Commands.parallel(
                          climbEntryCommandFactory.get(),
                          Commands.sequence(
                              Commands.runOnce(
                                  () -> superstructure.setWantedSuperState(WantedSuperState.CLIMB)),
                              climber.extendCommand())),
                      // Phase 3: Retract climber to pull robot up
                      climber.retractCommand(),
                      // Phase 4: Final small forward nudge
                      drivetrain
                          .applyRequest(() -> finalForwardRequest.withSpeeds(
                              new ChassisSpeeds(finalForwardSpeedMps, 0, 0)))
                          .withTimeout(finalForwardTimeSec),
                      Commands.waitSeconds(postMoveSettleSeconds))
                  .withName("Auto Climb Sequence");
            },
            Set.of(drivetrain, climber, superstructure)));

    // ======== UNJAM / EJECT (through Superstructure) ========
    // B - Hold reverse intake + indexer
    operator
        .b()
        .onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.UNJAM)))
        .onFalse(Commands.runOnce(() -> {
          if (superstructure.getWantedSuperState() == WantedSuperState.UNJAM) {
            superstructure.setWantedSuperState(WantedSuperState.IDLE);
          }
        }));

    // ==================== SHOOTER PIVOT ====================
    // Superstructure manages the shooter pivot in AIM/SHOOT states.
    // Manual override and homing bypass the Superstructure via the override flag.

    // Left Bumper - Manual override (operator left stick Y)
    operator
        .leftBumper()
        .onTrue(Commands.runOnce(() -> superstructure.setShooterPivotOverride(true)))
        .whileTrue(shooterPivot.manualControlCommand(negate(operator::getLeftY)))
        .onFalse(Commands.runOnce(() -> superstructure.setShooterPivotOverride(false)));

    // Right Bumper - Hold to control shooter pivot with left stick Y
    operator
        .rightBumper()
        .onTrue(Commands.runOnce(() -> superstructure.setShooterPivotOverride(true)))
        .whileTrue(shooterPivot.manualControlCommand(negate(operator::getLeftY)))
        .onFalse(Commands.runOnce(() -> superstructure.setShooterPivotOverride(false)));

    // X - Run shooter pivot homing routine (drives into hard stop to zero encoder)
    operator
        .x()
        .onTrue(Commands.sequence(
            Commands.runOnce(() -> superstructure.setShooterPivotOverride(true)),
            shooterPivot.homeCommand(),
            Commands.runOnce(() -> superstructure.setShooterPivotOverride(false))));

    final double rpmStep = 25.0;
    final double angleStepDeg = 0.25;

    SmartDashboard.putString("Tuning/Shooter/ActiveMode", "RPM");
    SmartDashboard.putNumber("Tuning/Shooter/RpmStep", rpmStep);
    SmartDashboard.putNumber("Tuning/Shooter/AngleStepDeg", angleStepDeg);

    // D-Pad LEFT/RIGHT tuning: nudge RPM up/down.
    operator.povLeft().onTrue(Commands.runOnce(() -> {
      double rawDistanceMeters = hubDistanceSupplier.get().in(Meters);
      double tuningDistanceMeters = ShooterInterpolationTable.getClosestRPMKey(rawDistanceMeters);
      double currentRpm =
          ShooterInterpolationTable.getRPM(Meters.of(tuningDistanceMeters)).in(RPM);
      double newRpm = currentRpm + rpmStep;
      ShooterInterpolationTable.hotSwapRPMValues(tuningDistanceMeters, newRpm);
      SmartDashboard.putString("Tuning/Shooter/LastChange", "RPM +");
      SmartDashboard.putNumber("Tuning/Shooter/RawDistanceMeters", rawDistanceMeters);
      SmartDashboard.putNumber("Tuning/Shooter/DistanceKeyMeters", tuningDistanceMeters);
      SmartDashboard.putNumber("Tuning/Shooter/NewRpm", newRpm);
    }));
    operator.povRight().onTrue(Commands.runOnce(() -> {
      double rawDistanceMeters = hubDistanceSupplier.get().in(Meters);
      double tuningDistanceMeters = ShooterInterpolationTable.getClosestRPMKey(rawDistanceMeters);
      double currentRpm =
          ShooterInterpolationTable.getRPM(Meters.of(tuningDistanceMeters)).in(RPM);
      double newRpm = currentRpm - rpmStep;
      ShooterInterpolationTable.hotSwapRPMValues(tuningDistanceMeters, newRpm);
      SmartDashboard.putString("Tuning/Shooter/LastChange", "RPM -");
      SmartDashboard.putNumber("Tuning/Shooter/RawDistanceMeters", rawDistanceMeters);
      SmartDashboard.putNumber("Tuning/Shooter/DistanceKeyMeters", tuningDistanceMeters);
      SmartDashboard.putNumber("Tuning/Shooter/NewRpm", newRpm);
    }));

    // ========== FORCE SHOOT OVERRIDE (through Superstructure) ==========
    // Right Trigger - Force-feed the shooter, bypassing on-target gates.
    operator
        .rightTrigger(0.5)
        .onTrue(Commands.runOnce(
            () -> superstructure.setWantedSuperState(WantedSuperState.FORCE_SHOOT)))
        .onFalse(Commands.runOnce(() -> {
          if (superstructure.getWantedSuperState() == WantedSuperState.FORCE_SHOOT) {
            superstructure.setWantedSuperState(WantedSuperState.IDLE);
          }
        }));

    // ======== CLIMB (through Superstructure + direct climber commands) ========
    // Start + Back together -> Set Superstructure to CLIMB and extend climber to
    // max position.
    // Works from any state (idle or retracted) to allow re-extension.
    new Trigger(() -> operator.start().getAsBoolean() && operator.back().getAsBoolean())
        .onTrue(Commands.sequence(
            Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.CLIMB)),
            climber.extendCommand()));

    // A button (while in climb mode and extended) -> Retract to climb position
    // Use onTrue on just A button, then conditionally run retract.
    // This prevents auto-triggering when isExtended() becomes true while A is held.
    operator
        .a()
        .onTrue(Commands.either(
            climber.retractCommand(),
            Commands.none(),
            () -> superstructure.isClimbing() && climber.isExtended()));

    // Back alone (not with Start) -> Abort climb from any state
    operator
        .back()
        .and(() -> !operator.start().getAsBoolean())
        .and(() -> superstructure.isClimbing())
        .onTrue(Commands.sequence(
            climber.abortCommand(),
            Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.IDLE))));
  }

  private static DoubleSupplier negate(DoubleSupplier supplier) {
    return () -> -supplier.getAsDouble();
  }
}
