// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.ShooterInterpolationTable;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.statemachine.FuelState;
import frc.robot.statemachine.HubShiftState;
import frc.robot.statemachine.MatchState;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
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
   */
  public static void configure(
      CommandXboxController operator,
      Superstructure superstructure,
      ShooterPivotSubsystem shooterPivot,
      ClimberSubsystem climber,
      RobotStateMachine stateMachine,
      Supplier<ShooterSetpoint> setpointSupplier,
      Supplier<Distance> hubDistanceSupplier) {

    // ==================== INVENTORY ====================
    // Y - Human-in-the-loop toggle EMPTY <-> LOADED
    operator
        .y()
        .onTrue(Commands.runOnce(() -> stateMachine.setFuelState(
            stateMachine.getFuelState() == FuelState.LOADED ? FuelState.EMPTY : FuelState.LOADED)));

    // ==================== HUB OVERRIDES ====================
    // D-Pad Up - Force hub active (offense)
    operator
        .povUp()
        .onTrue(Commands.runOnce(() -> stateMachine.setHubShiftState(HubShiftState.MY_HUB_ACTIVE)));

    // D-Pad Down - Force hub inactive (defense/hoard)
    operator
        .povDown()
        .onTrue(
            Commands.runOnce(() -> stateMachine.setHubShiftState(HubShiftState.MY_HUB_INACTIVE)));

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

    // ======== CLIMB SAFETY (through Superstructure + direct climber commands)
    // ========
    // Start + Back together -> Arm endgame and begin extending climber (safety
    // interlock)
    new Trigger(() -> operator.start().getAsBoolean() && operator.back().getAsBoolean())
        .onTrue(Commands.sequence(
            Commands.runOnce(() -> {
              stateMachine.setMatchState(MatchState.ENDGAME);
            }),
            Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.CLIMB)),
            climber.extendCommand()));

    // A button (while in climb mode) -> Trigger retract/climb phase after hooking
    // on bar
    operator
        .a()
        .and(() -> superstructure.isClimbing() && climber.isExtended())
        .onTrue(climber.climbCommand());

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
