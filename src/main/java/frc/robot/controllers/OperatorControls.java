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
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.commands.ShooterFactory;
import frc.robot.lib.ShooterInterpolationTable;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.statemachine.ClimbState;
import frc.robot.statemachine.FuelState;
import frc.robot.statemachine.GameState;
import frc.robot.statemachine.HubShiftState;
import frc.robot.statemachine.MatchState;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.Supplier;

/** Operator controller bindings (Port 1). Handles state-machine management and safety overrides. */
public final class OperatorControls {

  private OperatorControls() {} // Static utility class

  /**
   * Bind all operator controls.
   *
   * @param operator the operator's Xbox controller
   * @param intake intake wheels subsystem
   * @param pivot pivot arm subsystem
   * @param indexer indexer subsystem
   * @param climber climber subsystem
   * @param shooter shooter flywheel subsystem (for force-shoot override)
   * @param shooterPivot shooter pivot subsystem
   * @param stateMachine global robot state machine
   * @param setpointSupplier memoized distance-based setpoint supplier
   */
  public static void configure(
      CommandXboxController operator,
      IntakeWheelsSubsystem intake,
      PivotSubsystem pivot,
      frc.robot.subsystems.indexer.IndexerSubsystem indexer,
      ClimberSubsystem climber,
      ShooterSubsystem shooter,
      ShooterPivotSubsystem shooterPivot,
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

    // ==================== UNJAM / EJECT ====================
    // B - Hold reverse intake + indexer
    operator
        .b()
        .whileTrue(Commands.startEnd(
                () -> pivot.setWantedState(PivotSubsystem.WantedState.DEPLOY),
                () -> pivot.setWantedState(PivotSubsystem.WantedState.STOW),
                pivot)
            .alongWith(intake.intakeOutCommand(), indexer.reverseCommand()));

    // ==================== SHOOTER PIVOT ====================
    // Default: auto-aim tracking from distance-based setpoint
    // The pivot continuously tracks the angle from the interpolation table.
    shooterPivot.setDefaultCommand(shooterPivot.trackAngleCommand(() -> {
      ShooterSetpoint sp = setpointSupplier.get();
      return (sp != null && sp.isValid()) ? sp.pivotAngle() : ShooterPivotConstants.MIN_ANGLE;
    }));

    // Left Bumper - Manual override (operator left stick Y)
    operator.leftBumper().whileTrue(shooterPivot.manualControlCommand(() -> -operator.getLeftY()));

    // Right Bumper - Hold to control shooter pivot with left stick Y
    operator.rightBumper().whileTrue(shooterPivot.manualControlCommand(() -> -operator.getLeftY()));

    // X - Run shooter pivot homing routine (drives into hard stop to zero encoder)
    operator.x().onTrue(shooterPivot.homeCommand());

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

    // D-Pad LEFT/RIGHT tuning: nudge angle up/down.
    // SmartDashboard.putString("Tuning/Shooter/ActiveMode", "ANGLE");
    // operator
    // .povLeft()
    // .onTrue(Commands.runOnce(() -> {
    // double rawDistanceMeters = hubDistanceSupplier.get().in(Meters);
    // double tuningDistanceMeters =
    // ShooterInterpolationTable.getClosestAngleKey(rawDistanceMeters);
    // double currentAngle =
    // ShooterInterpolationTable.getAngle(Meters.of(tuningDistanceMeters)).in(Degrees);
    // double newAngle = currentAngle + angleStepDeg;
    // ShooterInterpolationTable.hotSwapAngleValues(tuningDistanceMeters, newAngle);
    // SmartDashboard.putString("Tuning/Shooter/LastChange", "ANGLE +");
    // SmartDashboard.putNumber("Tuning/Shooter/RawDistanceMeters",
    // rawDistanceMeters);
    // SmartDashboard.putNumber("Tuning/Shooter/DistanceKeyMeters",
    // tuningDistanceMeters);
    // SmartDashboard.putNumber("Tuning/Shooter/NewAngleDeg", newAngle);
    // }));
    // operator
    // .povRight()
    // .onTrue(Commands.runOnce(() -> {
    // double rawDistanceMeters = hubDistanceSupplier.get().in(Meters);
    // double tuningDistanceMeters =
    // ShooterInterpolationTable.getClosestAngleKey(rawDistanceMeters);
    // double currentAngle =
    // ShooterInterpolationTable.getAngle(Meters.of(tuningDistanceMeters)).in(Degrees);
    // double newAngle = currentAngle - angleStepDeg;
    // ShooterInterpolationTable.hotSwapAngleValues(tuningDistanceMeters, newAngle);
    // SmartDashboard.putString("Tuning/Shooter/LastChange", "ANGLE -");
    // SmartDashboard.putNumber("Tuning/Shooter/RawDistanceMeters",
    // rawDistanceMeters);
    // SmartDashboard.putNumber("Tuning/Shooter/DistanceKeyMeters",
    // tuningDistanceMeters);
    // SmartDashboard.putNumber("Tuning/Shooter/NewAngleDeg", newAngle);
    // }));

    // ==================== FORCE SHOOT OVERRIDE ====================
    // Right Trigger - Force-feed the shooter, bypassing on-target gates.
    // Use when the robot thinks it can't make the shot but the operator
    // disagrees (e.g., out-of-range or heading misalignment).
    operator
        .rightTrigger(0.5)
        .whileTrue(ShooterFactory.forceShoot(setpointSupplier, shooter, shooterPivot, indexer)
            .withName("Operator Force Shoot"));

    // ==================== CLIMB SAFETY ====================
    // Start + Back together -> L1 climb sequence arm (safety interlock)
    new Trigger(() -> operator.start().getAsBoolean() && operator.back().getAsBoolean())
        .onTrue(Commands.sequence(
            Commands.runOnce(() -> {
              stateMachine.setMatchState(MatchState.ENDGAME);
              stateMachine.setGameState(GameState.CLIMBING);
              stateMachine.setClimbState(ClimbState.CLIMBING_L1);
            }),
            climber.extendCommand()));
  }
}
