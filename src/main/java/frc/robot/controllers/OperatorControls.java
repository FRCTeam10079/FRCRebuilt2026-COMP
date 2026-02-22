// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import java.util.function.Supplier;

/**
 * Operator controller bindings (Port 1). Handles state-machine management and
 * safety overrides.
 */
public final class OperatorControls {

    private OperatorControls() {
    } // Static utility class

    /**
     * Bind all operator controls.
     *
     * @param operator     the operator's Xbox controller
     * @param intake       intake wheels subsystem
     * @param pivot        pivot arm subsystem
     * @param indexer      indexer subsystem
     * @param climber      climber subsystem
     * @param stateMachine global robot state machine
     */
    public static void configure(
            CommandXboxController operator,
            IntakeWheelsSubsystem intake,
            PivotSubsystem pivot,
            frc.robot.subsystems.indexer.IndexerSubsystem indexer,
            ClimberSubsystem climber,
            ShooterPivotSubsystem shooterPivot,
            RobotStateMachine stateMachine,
            Supplier<ShooterSetpoint> setpointSupplier) {

        // ==================== INVENTORY ====================
        // Y - Human-in-the-loop toggle EMPTY <-> LOADED
        operator
                .y()
                .toggleOnTrue(Commands.runOnce(() -> stateMachine.setFuelState(
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
                .whileTrue(Commands.startEnd(pivot::deployPivot, pivot::stowPivot, pivot)
                        .alongWith(intake.intakeOutCommand(), indexer.reverseCommand()));

        shooterPivot.setDefaultCommand(
                shooterPivot.trackAngleCommand(() -> {
                    ShooterSetpoint sp = setpointSupplier.get();
                    return (sp != null && sp.isValid())
                            ? sp.getPivotAngleDegrees()
                            : frc.robot.Constants.ShooterPivotConstants.MIN_ANGLE_DEGREES;
                }));

        operator
                .leftBumper()
                .whileTrue(shooterPivot.manualControlCommand(() -> -operator.getLeftY()));

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
