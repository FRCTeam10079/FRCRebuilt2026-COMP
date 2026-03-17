// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private Angle m_pivotSetpoint;
  // Stall detection state (only active while stowing)
  private boolean m_isStowing = false;
  private boolean m_isStalled = false;
  private final Timer m_stallTimer = new Timer();

  private boolean m_isIdle = false;
  private final Timer m_atSetpointTimer = new Timer();

  public PivotSubsystem(PivotIO io) {
    this.io = io;
    // Prevent pivot from moving on startup - set setpoint to current position
    m_pivotSetpoint = getPivotPosition();
  }

  public void deployPivot() {
    m_isStowing = false;
    m_isStalled = false;
    wakeFromIdle();
    setPivotSetpoint(IntakeConstants.Pivot.INTAKE_POSITION);
  }

  public void stowPivot() {
    m_isStowing = true;
    m_isStalled = false;
    m_stallTimer.stop();
    m_stallTimer.reset();
    setPivotSetpoint(IntakeConstants.Pivot.STOWED_POSITION);
  }

  private void setPivotSetpoint(Angle position) {
    m_pivotSetpoint = position;
  }

  private void wakeFromIdle() {
    m_isIdle = false;
    m_atSetpointTimer.stop();
  }

  public boolean isStalled() {
    return m_isStalled;
  }

  public boolean reachedSetpoint() {
    return getPivotPosition().isNear(m_pivotSetpoint, IntakeConstants.Pivot.DEPLOY_TOLERANCE);
  }

  public Angle getPivotPosition() {
    return Rotations.of(inputs.positionRotations);
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

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    detectStall();
    detectAtSetpoint();

    if (!m_isStowing && m_isIdle) {
      io.setNeutral();
    } else {
      io.setMotionMagicPosition(m_pivotSetpoint);
    }

    Logger.recordOutput("IntakePivot/setpoint", m_pivotSetpoint.in(Rotations));
    Logger.recordOutput("IntakePivot/position", getPivotPosition().in(Rotations));
    Logger.recordOutput("IntakePivot/reachedSetpoint", reachedSetpoint());
    Logger.recordOutput("IntakePivot/isStalled", m_isStalled);
    Logger.recordOutput("IntakePivot/isIdle", m_isIdle);
    Logger.recordOutput("IntakePivot/statorCurrent", inputs.statorCurrentAmps);
  }

  private void detectAtSetpoint() {
    if (m_isStowing || m_isIdle) {
      return;
    }

    if (!reachedSetpoint()) {
      m_atSetpointTimer.restart();
      return;
    }

    if (m_atSetpointTimer.hasElapsed(IntakeConstants.Pivot.IDLE_DEBOUNCE_TIME)) {
      m_isIdle = true;
    }
  }

  private void detectStall() {
    if (!m_isStowing || m_isStalled || reachedSetpoint()) {
      return;
    }

    if (Amps.of(inputs.statorCurrentAmps).lte(IntakeConstants.Pivot.STALL_CURRENT_THRESHOLD)) {
      m_stallTimer.stop();
      m_stallTimer.reset();
      return;
    }

    if (!m_stallTimer.isRunning()) {
      m_stallTimer.start();
    }

    if (m_stallTimer.hasElapsed(IntakeConstants.Pivot.STALL_TIME_THRESHOLD)) {
      m_isStalled = true;
      m_pivotSetpoint = getPivotPosition();
    }
  }

  public Command deployCommand() {
    return runOnce(this::deployPivot)
        .andThen(Commands.waitUntil(this::reachedSetpoint))
        .withName("Pivot Deploy");
  }

  public Command stowCommand() {
    return runOnce(this::stowPivot)
        .andThen(Commands.waitUntil(this::reachedSetpoint))
        .withName("Pivot Stow");
  }
}
