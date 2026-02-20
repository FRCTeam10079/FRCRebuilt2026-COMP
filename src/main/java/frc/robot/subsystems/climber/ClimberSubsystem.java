// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Stub subsystem for the climber mechanism.
 *
 * <p>Hardware has not been wired yet. All command factories currently return
 * {@code Commands.none()} so the rest of the robot code can reference them without null-checks.
 * Replace the stubs with real motor control once the climber is physically installed.
 */
public class ClimberSubsystem extends SubsystemBase {

  /** Create a new ClimberSubsystem. Motor configuration is deferred until hardware is wired. */
  public ClimberSubsystem() {
    configureMotors();
  }

  /** Configure climber motor(s). TODO: add TalonFX setup once hardware is ready. */
  private void configureMotors() {
    // TODO: instantiate TalonFX with ClimberConstants.CLIMBER_MOTOR_ID on
    // CLIMBER_CANBUS
  }

  // ==================== COMMAND FACTORIES ====================

  /** Extend the climber arm. TODO: replace with real motor control. */
  public Command extendCommand() {
    return Commands.none().withName("Climber Extend (stub)");
  }

  /** Retract the climber arm. TODO: replace with real motor control. */
  public Command retractCommand() {
    return Commands.none().withName("Climber Retract (stub)");
  }

  /** Stop the climber. TODO: replace with real motor control. */
  public Command stopCommand() {
    return Commands.none().withName("Climber Stop (stub)");
  }

  @Override
  public void periodic() {
    // TODO: publish climber telemetry (position, current, etc.)
  }
}
