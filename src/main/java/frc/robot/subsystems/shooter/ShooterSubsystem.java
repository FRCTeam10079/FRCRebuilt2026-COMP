// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private AngularVelocity m_targetRPM = RPM.zero();
  private boolean m_isEnabled = false;

  // Stability tracking with debouncing
  private int m_stabilityCounter = 0;

  private final SysIdRoutine m_sysIdRoutine;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;

    m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Read current velocity from logged inputs
    AngularVelocity currentRPM = RotationsPerSecond.of(inputs.masterVelocityRPS);

    // Update stability counter (debouncing logic)
    boolean isTargetPositive = m_targetRPM.gt(RPM.zero());
    if (m_isEnabled && isTargetPositive) {
      if (currentRPM.isNear(m_targetRPM, ShooterConstants.SHOOTER_SPEED_TOLERANCE)) {
        m_stabilityCounter =
            Math.min(m_stabilityCounter + 1, ShooterConstants.STABILITY_CYCLES_REQUIRED);
      } else {
        m_stabilityCounter = 0;
      }
    } else {
      m_stabilityCounter = 0;
    }

    // Apply control to motors
    if (m_isEnabled && isTargetPositive) {
      io.setVelocity(m_targetRPM.in(RotationsPerSecond));
    } else {
      io.stop();
    }

    // Telemetry via AdvantageKit
    Logger.recordOutput("Shooter/TargetRPM", m_targetRPM.in(RPM));
    Logger.recordOutput("Shooter/CurrentRPM", currentRPM.in(RPM));
    Logger.recordOutput("Shooter/ErrorRPM", m_targetRPM.minus(currentRPM).in(RPM));
    Logger.recordOutput("Shooter/IsEnabled", m_isEnabled);
    Logger.recordOutput("Shooter/IsReady", isReady());
    Logger.recordOutput("Shooter/StabilityCounter", m_stabilityCounter);
  }

  private void setTargetRPM(AngularVelocity rpm) {
    AngularVelocity clampedRPM =
        Constants.clamp(rpm, RPM.zero(), ShooterConstants.SHOOTER_MAX_SPEED);

    if (!clampedRPM.isNear(m_targetRPM, ShooterConstants.SHOOTER_SPEED_TOLERANCE)) {
      m_stabilityCounter = 0;
    }

    m_targetRPM = clampedRPM;
    m_isEnabled = clampedRPM.gt(RPM.zero());
  }

  private void spinUp() {
    setTargetRPM(ShooterConstants.SHOOTER_SPINUP_SPEED);
  }

  public void stop() {
    m_targetRPM = RPM.zero();
    m_isEnabled = false;
    m_stabilityCounter = 0;
  }

  public boolean isReady() {
    return m_isEnabled
        && m_targetRPM.gt(RPM.zero())
        && m_stabilityCounter >= ShooterConstants.STABILITY_CYCLES_REQUIRED;
  }

  public boolean isAtSetpoint() {
    if (!m_isEnabled || m_targetRPM.lt(RPM.zero())) {
      return false;
    }
    return m_targetRPM.isNear(
        RotationsPerSecond.of(inputs.masterVelocityRPS), ShooterConstants.SHOOTER_SPEED_TOLERANCE);
  }

  public AngularVelocity getCurrentRPM() {
    return RotationsPerSecond.of(inputs.masterVelocityRPS);
  }

  public AngularVelocity getTargetRPM() {
    return m_targetRPM;
  }

  public boolean isEnabled() {
    return m_isEnabled;
  }

  public double getMasterSupplyCurrentAmps() {
    return inputs.masterSupplyCurrentAmps;
  }

  public double getSlaveSupplyCurrentAmps() {
    return inputs.slaveSupplyCurrentAmps;
  }

  public double getMasterStatorCurrentAmps() {
    return inputs.masterStatorCurrentAmps;
  }

  public double getSlaveStatorCurrentAmps() {
    return inputs.slaveStatorCurrentAmps;
  }

  public double getMasterVoltageVolts() {
    return inputs.masterVoltageVolts;
  }

  public double getSlaveVoltageVolts() {
    return inputs.slaveVoltageVolts;
  }

  public Command spinUpCommand() {
    return runOnce(this::spinUp).withName("Shooter Spin Up");
  }

  public Command spinUpAndWaitCommand() {
    return run(this::spinUp).until(this::isReady).withName("Shooter Spin Up & Wait");
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Shooter Stop");
  }

  public Command holdRPMCommand(AngularVelocity rpm) {
    return startEnd(() -> setTargetRPM(rpm), this::stop).withName("Shooter Hold " + rpm + " RPM");
  }

  public Command holdRPMCommand(Supplier<AngularVelocity> rpmSupplier) {
    return run(() -> setTargetRPM(rpmSupplier.get()))
        .finallyDo(interrupted -> stop())
        .withName("Shooter Dynamic RPM");
  }

  public boolean isAt(AngularVelocity target) {
    if (target.lt(RPM.zero())) return false;
    return getCurrentRPM().isNear(target, ShooterConstants.ON_TARGET_RPM_PERCENT);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
