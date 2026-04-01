// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.ShooterInterpolationTable;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.statemachine.ClimbState;
import frc.robot.statemachine.FuelState;
import frc.robot.statemachine.MatchState;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Operator controller bindings (Port 1). State-machine management, safety overrides, and shooter
 * pivot manual control. Mechanism actions route through the {@link Superstructure}.
 */
public final class OperatorControls {

  private static final double SHOOTER_TUNING_EVENT_CLEAR_DELAY_SECONDS = 0.005;

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

    final AtomicInteger tuningEventCounter = new AtomicInteger(0);

    // ==================== INVENTORY ====================
    // Y - Human-in-the-loop toggle EMPTY <-> LOADED
    operator
        .y()
        .onTrue(Commands.runOnce(() -> stateMachine.setFuelState(
            stateMachine.getFuelState() == FuelState.LOADED ? FuelState.EMPTY : FuelState.LOADED)));

    // ======== UNJAM / EJECT (through Superstructure) ========
    // B - Hold reverse intake + indexer
    operator
        .b()
        .onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.UNJAM)))
        .onFalse(updateWantedStateFromOperatorInputs(operator, superstructure));

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
    final double confirmRumbleSeconds = 0.15;
    final double confirmRumbleStrength = 0.8;

    // D-Pad tuning offsets: up/down = angle, left/right = RPM.
    operator.povUp().onTrue(Commands.runOnce(() -> {
      ShooterSetpoint.adjustAngleOffset(Degrees.of(angleStepDeg));
      logAdjustmentEvent(
          tuningEventCounter, "AdjustAngle", hubDistanceSupplier.get(), 0.0, angleStepDeg);
    }));

    operator.povDown().onTrue(Commands.runOnce(() -> {
      ShooterSetpoint.adjustAngleOffset(Degrees.of(-angleStepDeg));
      logAdjustmentEvent(
          tuningEventCounter, "AdjustAngle", hubDistanceSupplier.get(), 0.0, -angleStepDeg);
    }));

    operator.povLeft().onTrue(Commands.runOnce(() -> {
      ShooterSetpoint.adjustRPMOffset(RPM.of(rpmStep));
      logAdjustmentEvent(tuningEventCounter, "AdjustRPM", hubDistanceSupplier.get(), rpmStep, 0.0);
    }));

    operator.povRight().onTrue(Commands.runOnce(() -> {
      ShooterSetpoint.adjustRPMOffset(RPM.of(-rpmStep));
      logAdjustmentEvent(tuningEventCounter, "AdjustRPM", hubDistanceSupplier.get(), -rpmStep, 0.0);
    }));

    // A - Confirm current correction: insert exact-distance table entries and clear
    // offsets.
    operator
        .a()
        .onTrue(Commands.sequence(
            Commands.runOnce(() -> {
              Distance distance = hubDistanceSupplier.get();
              double distanceMeters = distance.in(Meters);

              double rawTableRpm = ShooterInterpolationTable.getRPM(distance).in(RPM);
              double rawTableAngleDeg =
                  ShooterInterpolationTable.getAngle(distance).in(Degrees);
              double appliedOffsetRpm = ShooterSetpoint.getRPMOffset().in(RPM);
              double appliedOffsetAngleDeg = ShooterSetpoint.getAngleOffset().in(Degrees);

              ShooterSetpoint correctedSetpoint = ShooterSetpoint.fromDistance(distance);
              double insertedRpm = correctedSetpoint.flywheelRPM().in(RPM);
              double insertedAngleDeg = correctedSetpoint.pivotAngle().in(Degrees);

              ShooterInterpolationTable.hotSwapRPMValues(distanceMeters, insertedRpm);
              ShooterInterpolationTable.hotSwapAngleValues(distanceMeters, insertedAngleDeg);

              ShooterSetpoint.resetOffsets();

              int eventSeq = tuningEventCounter.incrementAndGet();
              Logger.recordOutput("ShooterTuning/EventSequence", eventSeq);
              Logger.recordOutput("ShooterTuning/EventType", "ConfirmInsert");
              Logger.recordOutput("ShooterTuning/DistanceMeters", distanceMeters);
              Logger.recordOutput("ShooterTuning/RawTableRPM", rawTableRpm);
              Logger.recordOutput("ShooterTuning/RawTableAngleDeg", rawTableAngleDeg);
              Logger.recordOutput("ShooterTuning/AppliedOffsetRPM", appliedOffsetRpm);
              Logger.recordOutput("ShooterTuning/AppliedOffsetAngleDeg", appliedOffsetAngleDeg);
              Logger.recordOutput("ShooterTuning/FinalCommandRPM", insertedRpm);
              Logger.recordOutput("ShooterTuning/FinalCommandAngleDeg", insertedAngleDeg);
              Logger.recordOutput("ShooterTuning/SetpointValid", correctedSetpoint.isValid());
              Logger.recordOutput("ShooterTuning/InsertedRPM", insertedRpm);
              Logger.recordOutput("ShooterTuning/InsertedAngleDeg", insertedAngleDeg);
              Logger.recordOutput(
                  "ShooterTuning/OffsetRPMAfterReset",
                  ShooterSetpoint.getRPMOffset().in(RPM));
              Logger.recordOutput(
                  "ShooterTuning/OffsetAngleDegAfterReset",
                  ShooterSetpoint.getAngleOffset().in(Degrees));
              Logger.recordOutput("ShooterTuning/OffsetsReset", true);
              scheduleShooterTuningEventClear();
            }),
            rumblePulseCommand(operator, confirmRumbleStrength, confirmRumbleSeconds)));

    // ========== FORCE SHOOT OVERRIDE (through Superstructure) ==========
    // Right Trigger - Force-feed the shooter, bypassing on-target gates.
    operator
        .rightTrigger(0.5)
        .onTrue(Commands.runOnce(
            () -> superstructure.setWantedSuperState(WantedSuperState.FORCE_SHOOT)))
        .onFalse(updateWantedStateFromOperatorInputs(operator, superstructure));

    // ======== CLIMB SAFETY (through Superstructure) ========
    // Start + Back together -> L1 climb sequence arm (safety interlock)
    new Trigger(() -> operator.start().getAsBoolean() && operator.back().getAsBoolean())
        .onTrue(Commands.sequence(
            Commands.runOnce(() -> {
              stateMachine.setMatchState(MatchState.ENDGAME);
              stateMachine.setClimbState(ClimbState.CLIMBING_L1);
            }),
            Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.CLIMB))));
  }

  private static DoubleSupplier negate(DoubleSupplier supplier) {
    return () -> -supplier.getAsDouble();
  }

  private static Command updateWantedStateFromOperatorInputs(
      CommandXboxController operator, Superstructure superstructure) {
    return Commands.runOnce(() -> {
      WantedSuperState desiredState = WantedSuperState.IDLE;

      if (operator.rightTrigger(0.5).getAsBoolean()) {
        desiredState = WantedSuperState.FORCE_SHOOT;
      } else if (operator.b().getAsBoolean()) {
        desiredState = WantedSuperState.UNJAM;
      }

      superstructure.setWantedSuperState(desiredState);
    });
  }

  private static Command rumblePulseCommand(
      CommandXboxController operator, double strength, double seconds) {
    return Commands.startEnd(
            () -> operator.getHID().setRumble(RumbleType.kBothRumble, strength),
            () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0.0))
        .withTimeout(seconds);
  }

  private static void scheduleShooterTuningEventClear() {
    CommandScheduler.getInstance()
        .schedule(Commands.waitSeconds(SHOOTER_TUNING_EVENT_CLEAR_DELAY_SECONDS)
            .andThen(Commands.runOnce(() -> Logger.recordOutput("ShooterTuning/EventType", ""))));
  }

  private static void logAdjustmentEvent(
      AtomicInteger eventCounter,
      String eventType,
      Distance distance,
      double deltaRPM,
      double deltaAngleDeg) {
    ShooterSetpoint correctedSetpoint = ShooterSetpoint.fromDistance(distance);

    Logger.recordOutput("ShooterTuning/EventSequence", eventCounter.incrementAndGet());
    Logger.recordOutput("ShooterTuning/EventType", eventType);
    Logger.recordOutput("ShooterTuning/DistanceMeters", distance.in(Meters));
    Logger.recordOutput(
        "ShooterTuning/RawTableRPM", ShooterInterpolationTable.getRPM(distance).in(RPM));
    Logger.recordOutput(
        "ShooterTuning/RawTableAngleDeg",
        ShooterInterpolationTable.getAngle(distance).in(Degrees));
    Logger.recordOutput(
        "ShooterTuning/AppliedOffsetRPM", ShooterSetpoint.getRPMOffset().in(RPM));
    Logger.recordOutput(
        "ShooterTuning/AppliedOffsetAngleDeg", ShooterSetpoint.getAngleOffset().in(Degrees));
    Logger.recordOutput(
        "ShooterTuning/FinalCommandRPM", correctedSetpoint.flywheelRPM().in(RPM));
    Logger.recordOutput(
        "ShooterTuning/FinalCommandAngleDeg", correctedSetpoint.pivotAngle().in(Degrees));
    Logger.recordOutput("ShooterTuning/SetpointValid", correctedSetpoint.isValid());
    Logger.recordOutput("ShooterTuning/DeltaRPM", deltaRPM);
    Logger.recordOutput("ShooterTuning/DeltaAngleDeg", deltaAngleDeg);
    Logger.recordOutput("ShooterTuning/OffsetsReset", false);
    scheduleShooterTuningEventClear();
  }
}
