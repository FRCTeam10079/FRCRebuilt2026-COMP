// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Intake wheels subsystem for collecting game pieces.
 *
 * <p>Uses IO abstraction for hardware independence.
 */
public class IntakeWheelsSubsystem extends SubsystemBase {

  private final IntakeWheelsIO io;
  private final IntakeWheelsIOInputsAutoLogged inputs = new IntakeWheelsIOInputsAutoLogged();

  public IntakeWheelsSubsystem(IntakeWheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeWheels", inputs);
  }

  /** Run intake wheels inward to collect game pieces. */
  public void intakeIn() {
    io.setVelocity(IntakeConstants.Wheels.INTAKE_IN_RPM / 60.0);
  }

  /** Reverse intake wheels to eject stuck game pieces. */
  private void intakeOut() {
    io.setVelocity(IntakeConstants.Wheels.INTAKE_OUT_RPM / 60.0);
  }

  /** Stop the intake wheels. */
  public void stop() {
    io.stop();
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getStatorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  public double getMotorVoltageVolts() {
    return inputs.voltageVolts;
  }

  // ==================== COMMAND FACTORIES ====================

  public Command intakeInCommand() {
    return startEnd(this::intakeIn, this::stop).withName("Intake In");
  }

  public Command intakeOutCommand() {
    return startEnd(this::intakeOut, this::stop).withName("Intake Out");
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Intake Stop");
  }
}
