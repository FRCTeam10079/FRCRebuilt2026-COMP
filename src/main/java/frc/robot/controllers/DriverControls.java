// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.lib.ShooterInterpolationTable;
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

  private DriverControls() {
  } // Static utility class

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
    // NOTE: Each condition is evaluated into a separate variable to avoid Java
    // short-circuit && — previously isAtLaunchHeadingGoal() was never called
    // when the flywheel wasn't at speed, hiding heading-tracking diagnostics.
    int[] gateLogCounter = { 0 };
    Trigger sotmReady = new Trigger(() -> {
      LaunchParameters params = LaunchCalculator.getInstance().getParameters();
      if (params == null || !params.isValid())
        return false;

      boolean fly = shooter.isAt(RPM.of(params.flywheelRPM()));
      boolean piv = shooterPivot.isAtAngle(
          Degrees.of(params.pivotAngleDegrees()), ShooterPivotConstants.SHOOTING_TOLERANCE);
      boolean hdg = drivetrain.isAtLaunchHeadingGoal();

      // Periodic gate logging at ~2Hz with actual sensor values
      gateLogCounter[0]++;
      if (gateLogCounter[0] % 25 == 0) {
        double pitchDeg = Math.abs(drivetrain.getPigeon2().getPitch().getValueAsDouble());
        double rollDeg = Math.abs(drivetrain.getPigeon2().getRoll().getValueAsDouble());
        System.out.printf(
            "SOTM_GATE,%.3f,fly=%b(%.0f/%.0f),piv=%b(%.1f/%.1f),hdg=%b(%.1f),lvl(%s)(p=%.1f,r=%.1f)%n",
            Timer.getFPGATimestamp(),
            fly,
            params.flywheelRPM(),
            shooter.getCurrentRPM().in(RPM),
            piv,
            params.pivotAngleDegrees(),
            shooterPivot.getCurrentAngle().in(Degrees),
            hdg,
            params.driveAngle().minus(drivetrain.getState().Pose.getRotation()).getDegrees(),
            drivetrain.isLevelForLaunch() ? "ok" : "FAIL",
            pitchDeg,
            rollDeg);
      }

      return fly && piv && hdg;
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

    controller.b().onTrue(Commands.runOnce(() -> invertTranslation[0] = !invertTranslation[0]));

    // ==================== TOF TUNING (D-PAD LEFT/RIGHT) ====================
    // D-pad Left - Increase TOF at nearest distance key
    // D-pad Right - Decrease TOF at nearest distance key
    // Prints current distance bucket and TOF value on each press.
    controller.povLeft().onTrue(Commands.runOnce(() -> {
      double dist = ShooterMath.getDistanceToHub(drivetrain.getState().Pose).in(
          edu.wpi.first.units.Units.Meters);
      ShooterInterpolationTable.adjustTof(dist, true);
    }));
    controller.povRight().onTrue(Commands.runOnce(() -> {
      double dist = ShooterMath.getDistanceToHub(drivetrain.getState().Pose).in(
          edu.wpi.first.units.Units.Meters);
      ShooterInterpolationTable.adjustTof(dist, false);
    }));
    // D-pad Up - Print current TOF at nearest key (read-only check)
    controller.povUp().onTrue(Commands.runOnce(() -> {
      double dist = ShooterMath.getDistanceToHub(drivetrain.getState().Pose).in(
          edu.wpi.first.units.Units.Meters);
      ShooterInterpolationTable.printCurrentTof(dist);
    }));

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
