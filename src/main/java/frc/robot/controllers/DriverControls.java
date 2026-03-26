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
import frc.robot.commands.ShootOnTheMoveDrive;
import frc.robot.commands.ShooterFactory;
import frc.robot.lib.ShooterMath;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.statemachine.FuelState;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.Supplier;

/**
 * Driver controller bindings (Port 0). All mechanism actions route through the
 * {@link Superstructure} so it can coordinate subsystems. Drivetrain commands (heading lock, brake,
 * vision align) remain direct since the Superstructure does not manage the drivetrain.
 */
public final class DriverControls {

  private DriverControls() {} // Static utility class

  /**
   * Bind all driver controls.
   *
   * @param controller the driver's Xbox controller
   * @param drivetrain swerve drivetrain subsystem
   * @param vision vision subsystem (for alignment commands)
   * @param superstructure the Superstructure coordinator
   * @param stateMachine global robot state machine (for fuel feedback triggers)
   * @param setpointSupplier memoized distance-based setpoint supplier
   */
  public static void configure(
      CommandXboxController controller,
      CommandSwerveDrivetrain drivetrain,
      VisionSubsystem vision,
      Superstructure superstructure,
      RobotStateMachine stateMachine,
      Supplier<ShooterSetpoint> setpointSupplier) {

    // Toggle to invert driver translation controls (left stick X/Y).
    final boolean[] invertTranslation = {false};
    Supplier<Double> translationY =
        () -> invertTranslation[0] ? -controller.getLeftY() : controller.getLeftY();
    Supplier<Double> translationX =
        () -> invertTranslation[0] ? -controller.getLeftX() : controller.getLeftX();

    // ==================== DEFAULT DRIVE ====================
    drivetrain.setDefaultCommand(drivetrain.smoothTeleopDriveCommand(
        translationY::get,
        translationX::get,
        // controller::getLeftY,
        // controller::getLeftX,
        () -> controller.getRightX(),
        Constants.DrivetrainConstants.MAX_SPEED_MPS,
        Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // ==================== INTAKE (through Superstructure) ====================
    // Left Trigger - Hold to collect (deploy pivot + run intake wheels + index)
    // Release returns to IDLE. Superstructure handles subsystem coordination.
    controller
        .leftTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
        .onTrue(
            Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.COLLECT)))
        .onFalse(setIdleIfStill(superstructure, WantedSuperState.COLLECT));

    // ==================== SHOOTING (through Superstructure) ====================
    // Right Bumper - Hold to aim at hub (heading lock) + pre-spin via
    // Superstructure
    // Drivetrain heading lock is direct (Superstructure doesn't manage drivetrain).
    // Superstructure handles shooter + pivot coordination via AIM state.
    controller
        .rightBumper()
        .onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.AIM)))
        .onFalse(setIdleIfStill(superstructure, WantedSuperState.AIM));

    // Heading lock while RB is held (drivetrain-only, parallel to Superstructure
    // AIM)
    controller
        .rightBumper()
        .whileTrue(createAimAtHubCommand(drivetrain, translationY, translationX));

    // Right Trigger - Hold to force-shoot (aim + feed when flywheel ready)
    // When released: falls back to AIM if RB is still held, otherwise IDLE.
    controller
        .rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
        .onTrue(Commands.runOnce(
            () -> superstructure.setWantedSuperState(WantedSuperState.FORCE_SHOOT)))
        .onFalse(Commands.runOnce(() -> {
          if (controller.rightBumper().getAsBoolean()) {
            superstructure.setWantedSuperState(WantedSuperState.AIM);
          } else {
            superstructure.setWantedSuperState(WantedSuperState.IDLE);
          }
        }));

    // Rumble while actively shooting
    new Trigger(() -> superstructure.getCurrentSuperState()
                == Superstructure.CurrentSuperState.FORCE_SHOOTING
            || superstructure.getCurrentSuperState() == Superstructure.CurrentSuperState.SHOOTING)
        .and(controller.rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD))
        .onTrue(Commands.runOnce(() -> controller
            .getHID()
            .setRumble(RumbleType.kBothRumble, Constants.StateMachineConstants.RUMBLE_STRONG)))
        .onFalse(
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // ==================== VISION ALIGNMENT ====================
    // A - Align to AprilTag (CENTER)
    controller.a().whileTrue(new AlignToAprilTag(drivetrain, vision, AlignPosition.CENTER));

    controller.b().onTrue(Commands.runOnce(() -> invertTranslation[0] = !invertTranslation[0]));

    controller
        .leftBumper()
        .whileTrue(new ShootOnTheMoveDrive(
            drivetrain, vision, controller::getLeftY, controller::getLeftX));

    // ==================== STOW (through Superstructure) ====================
    // D-pad Down - Stow intake pivot
    controller
        .povDown()
        .onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.STOW)))
        .onFalse(restoreDriverHeldState(controller, superstructure));

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

  private static Command createAimAtHubCommand(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Double> translationY,
      Supplier<Double> translationX) {
    return ShooterFactory.aimAtHub(
        drivetrain,
        translationY::get,
        translationX::get,
        () -> ShooterMath.getHeadingToHub(drivetrain.getState().Pose),
        Constants.DrivetrainConstants.MAX_ALIGNING_SPEED_MPS,
        Constants.DrivetrainConstants.MAX_ALIGNING_ANGULAR_RATE_RAD_PER_SEC);
  }

  private static Command setIdleIfStill(
      Superstructure superstructure, WantedSuperState expectedState) {
    return Commands.runOnce(() -> {
      if (superstructure.getWantedSuperState() == expectedState) {
        superstructure.setWantedSuperState(WantedSuperState.IDLE);
      }
    });
  }

  private static Command restoreDriverHeldState(
      CommandXboxController controller, Superstructure superstructure) {
    return Commands.runOnce(() -> {
      // Rebuild intent from live button states because trigger onTrue is edge-only.
      if (controller
          .rightTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
          .getAsBoolean()) {
        superstructure.setWantedSuperState(WantedSuperState.FORCE_SHOOT);
      } else if (controller.rightBumper().getAsBoolean()) {
        superstructure.setWantedSuperState(WantedSuperState.AIM);
      } else if (controller
          .leftTrigger(Constants.ControllerConstants.TRIGGER_THRESHOLD)
          .getAsBoolean()) {
        superstructure.setWantedSuperState(WantedSuperState.COLLECT);
      } else {
        superstructure.setWantedSuperState(WantedSuperState.IDLE);
      }
    });
  }
}
