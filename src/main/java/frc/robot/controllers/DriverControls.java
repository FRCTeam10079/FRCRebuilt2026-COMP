// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AlignPosition;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.ShooterFactory;
import frc.robot.lib.LaunchCalculator;
import frc.robot.lib.LaunchCalculator.LaunchParameters;
import frc.robot.lib.ShooterMath;
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
        () -> controller.getRightX(),
        Constants.DrivetrainConstants.MAX_SPEED_MPS,
        Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // ==================== INTAKE ====================
    // Left Trigger - Hold to deploy pivot + run intake wheels
    // Release stops wheels but pivot stays deployed so balls
    // aren't disturbed. Press X to stow pivot when ready.
    controller
        .leftTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
        .whileTrue(
            Commands.startEnd(
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
            // .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
            );

    // ==================== SHOOTING (DISTANCE-BASED) ====================
    // Right Bumper - Hold to aim at hub (heading lock) + pre-spin + track pivot
    // angle
    // The driver controls translation while the drivetrain auto-rotates toward the
    // hub.
    controller
        .rightBumper()
        .whileTrue(ShooterFactory.aimAtHub(
                drivetrain,
                controller::getLeftY,
                controller::getLeftX,
                () -> ShooterMath.getHeadingToHub(drivetrain.getState().Pose),
                Constants.DrivetrainConstants.MAX_ALIGNING_SPEED_MPS,
                Constants.DrivetrainConstants.MAX_ALIGNING_ANGULAR_RATE_RAD_PER_SEC)
            .alongWith(ShooterFactory.aimAndSpinUp(setpointSupplier, shooter, shooterPivot))
            .beforeStarting(() -> stateMachine.setGameState(GameState.SCORING))
            .finallyDo(() -> {
              if (stateMachine.getGameState() == GameState.SCORING) {
                stateMachine.setGameState(GameState.IDLE);
              }
            }));

    // Right Trigger - Hold to shoot (waits for on-target, then auto-feeds)
    // Assumes aim-at-hub is engaged via right bumper, OR driver is manually aiming.
    controller
        .rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
        .whileTrue(ShooterFactory.forceShoot(setpointSupplier, shooter, shooterPivot, indexer)

            // align old
            // () -> {
            // Heading is "on target" when we're close to the hub bearing
            // Angle targetHeading =
            // ShooterMath.getHeadingToHub(drivetrain.getState().Pose);
            // Angle currentHeading = drivetrain.getState().Pose.getRotation().getMeasure();
            // return Constants.angleDistance(currentHeading, targetHeading)
            // .lte(Constants.ShooterConstants.HEADING_TOLERANCE);
            // }
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

    // ==================== SHOOT ON THE MOVE ====================
    // Left Bumper - Hold to activate SOTM: auto-aims heading while driving,
    // pre-spins the shooter from LaunchCalculator, and auto-feeds when all
    // conditions are met (valid parameters, heading + level, flywheel + pivot).
    // Adapted from MA (6328) pattern: drive + spin-up on button hold, debounced
    // feeding on compound trigger.

    // Step 1: SOTM drive (heading lock + velocity limiting + COR shifting)
    // + launcher spin-up run while left bumper is held.
    controller
        .leftBumper()
        .whileTrue(drivetrain
            .shootOnTheMoveDriveCommand(controller::getLeftY, controller::getLeftX)
            .alongWith(ShooterFactory.aimAndSpinUpFromLauncher(shooter, shooterPivot))
            .beforeStarting(() -> stateMachine.setGameState(GameState.SCORING))
            .finallyDo(() -> {
              if (stateMachine.getGameState() == GameState.SCORING) {
                stateMachine.setGameState(GameState.IDLE);
              }
            }));

    // Step 2: Debounced auto-feeding. All sub-conditions must hold, then stay
    // true for 0.25 s (falling debounce) before the indexer feeds. This prevents
    // feeding on a momentary flicker of all-OK.
    Trigger sotmReady = new Trigger(() -> {
      LaunchParameters params = LaunchCalculator.getInstance().getParameters();
      if (params == null || !params.isValid()) return false;
      return shooter.isAt(RPM.of(params.flywheelRPM()))
          && shooterPivot.isAtAngle(
              Degrees.of(params.pivotAngleDegrees()), ShooterPivotConstants.SHOOTING_TOLERANCE)
          && drivetrain.isAtLaunchHeadingGoal();
    });

    controller
        .leftBumper()
        .and(() -> {
          LaunchParameters params = LaunchCalculator.getInstance().getParameters();
          return params != null && params.isValid();
        })
        .and(sotmReady.debounce(0.25, DebounceType.kFalling))
        .whileTrue(indexer.feedCommand());

    // Rumble when SOTM is ready to fire (left bumper held + in tolerance)
    controller
        .leftBumper()
        .and(sotmReady)
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
    controller.povDown().whileTrue(pivot.stowCommand());

    // ==================== X-STANCE ====================
    // X - Hold defensive wheel lock
    SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    controller.x().whileTrue(drivetrain.applyRequest(() -> brakeRequest));

    // ==================== DRIVER FEEDBACK ====================
    // Pulse rumble while loaded so driver knows they can leave loading zone.
    // Suppressed while SOTM is active (left bumper held) so the steady SOTM
    // readiness rumble isn't overwritten by the pulse's silence phase.
    new Trigger(() -> stateMachine.getFuelState() == FuelState.LOADED)
        .and(controller.leftBumper().negate())
        .whileTrue(Commands.repeatingSequence(
            Commands.runOnce(() -> controller
                .getHID()
                .setRumble(RumbleType.kBothRumble, Constants.StateMachineConstants.RUMBLE_LIGHT)),
            Commands.waitSeconds(0.15),
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)),
            Commands.waitSeconds(0.45)));
  }
}
