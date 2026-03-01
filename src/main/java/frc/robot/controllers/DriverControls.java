// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AlignPosition;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.ShooterFactory;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.statemachine.FuelState;
import frc.robot.statemachine.GameState;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.Supplier;

/**
 * Driver controller bindings (Port 0). All driver button->command mappings live here so
 * RobotContainer stays lean.
 */
public final class DriverControls {

  private DriverControls() {} // Static utility class

  /**
   * Bind all driver controls.
   *
   * @param controller the driver's Xbox controller
   * @param drivetrain swerve drivetrain subsystem
   * @param vision vision subsystem (for alignment commands)
   * @param intake intake wheels subsystem
   * @param pivot intake pivot subsystem
   * @param shooter shooter subsystem
   * @param shooterPivot shooter pivot subsystem
   * @param indexer indexer subsystem
   * @param stateMachine global robot state machine
   * @param setpointSupplier memoized distance-based setpoint supplier
   */
  public static void configure(
      CommandXboxController controller,
      CommandSwerveDrivetrain drivetrain,
      VisionSubsystem vision,
      IntakeWheelsSubsystem intake,
      PivotSubsystem pivot,
      ShooterSubsystem shooter,
      ShooterPivotSubsystem shooterPivot,
      IndexerSubsystem indexer,
      RobotStateMachine stateMachine,
      Supplier<ShooterSetpoint> setpointSupplier) {

    // ==================== DEFAULT DRIVE ====================
    drivetrain.setDefaultCommand(drivetrain.smoothTeleopDriveCommand(
        controller::getLeftY,
        controller::getLeftX,
        () -> -controller.getRightX(),
        Constants.DrivetrainConstants.MAX_SPEED_MPS,
        Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // ==================== INTAKE ====================
    // Left Trigger - Hold to deploy pivot + run intake wheels
    // Release stops wheels but pivot stays deployed so balls
    // aren't disturbed. Press X to stow pivot when ready.
    controller
        .leftTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
        .whileTrue(Commands.startEnd(
                () -> {
                  pivot.deployPivot();
                  intake.intakeIn();
                  stateMachine.setGameState(GameState.COLLECTING);
                },
                () -> {
                  intake.stop();
                  if (stateMachine.getGameState() == GameState.COLLECTING) {
                    stateMachine.setGameState(GameState.IDLE);
                  }
                },
                intake)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

    // ==================== SHOOTING (SHOOT-ON-THE-MOVE) ====================
    // Right Bumper - Hold to engage shoot-on-the-move heading lock + pre-spin.
    // The drivetrain automatically rotates toward the predicted launch angle
    // (accounting for robot velocity + time-of-flight) while the driver retains
    // full translational control. Flywheel and pivot track LaunchCalculator output.
    controller
        .rightBumper()
        .whileTrue(drivetrain
            .shootOnTheMoveDriveCommand(controller::getLeftY, controller::getLeftX)
            .alongWith(ShooterFactory.aimAndSpinUpFromLauncher(shooter, shooterPivot))
            .beforeStarting(() -> stateMachine.setGameState(GameState.SCORING))
            .finallyDo(() -> {
              if (stateMachine.getGameState() == GameState.SCORING) {
                stateMachine.setGameState(GameState.IDLE);
              }
            }));

    // Right Trigger - Hold to fire (waits for launch conditions, then auto-feeds).
    // Uses LaunchCalculator-based setpoints and the wider heading tolerance (10
    // deg)
    // designed for shooting while moving.
    controller
        .rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
        .whileTrue(ShooterFactory.shootOnTheMove(
                setpointSupplier, shooter, shooterPivot, indexer, drivetrain)
            .beforeStarting(() -> stateMachine.setGameState(GameState.SCORING))
            .finallyDo(() -> {
              if (stateMachine.getGameState() == GameState.SCORING) {
                stateMachine.setGameState(GameState.IDLE);
              }
            })
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

    // Rumble while shooter is ready and right trigger is held
    new Trigger(shooter::isReady)
        .and(controller.rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD))
        .onTrue(Commands.runOnce(() -> controller
            .getHID()
            .setRumble(RumbleType.kBothRumble, Constants.StateMachineConstants.RUMBLE_STRONG)))
        .onFalse(
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // ==================== VISION ALIGNMENT ====================
    // A - Align to AprilTag (CENTER)
    controller.a().whileTrue(new AlignToAprilTag(drivetrain, vision, AlignPosition.CENTER));

    controller.b().onTrue(Commands.runOnce(drivetrain::resetFieldHeading));

    // ==================== STOW PIVOT ====================
    // D-pad Down - Stow intake pivot
    controller.povDown().onTrue(pivot.stowCommand());

    // ==================== X-STANCE ====================
    // X - Hold defensive wheel lock
    SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    controller.x().whileTrue(drivetrain.applyRequest(() -> brakeRequest));

    // ==================== DRIVER FEEDBACK ====================
    // Pulse rumble while loaded so driver knows they can leave loading zone
    new Trigger(() -> stateMachine.getFuelState() == FuelState.LOADED)
        .whileTrue(Commands.repeatingSequence(
            Commands.runOnce(() -> controller
                .getHID()
                .setRumble(RumbleType.kBothRumble, Constants.StateMachineConstants.RUMBLE_LIGHT)),
            Commands.waitSeconds(0.15),
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)),
            Commands.waitSeconds(0.45)));
  }
}
