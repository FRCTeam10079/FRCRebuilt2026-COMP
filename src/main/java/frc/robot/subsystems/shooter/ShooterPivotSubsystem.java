// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterPivotConstants;
import java.util.function.DoubleSupplier;

public class ShooterPivotSubsystem extends SubsystemBase {

  private final TalonFX m_pivotMotor;

  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  public ShooterPivotSubsystem() {
    m_pivotMotor = new TalonFX(ShooterPivotConstants.MOTOR_ID);
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(ShooterPivotConstants.SUPPLY_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(ShooterPivotConstants.STATOR_CURRENT_LIMIT);

    m_pivotMotor.getConfigurator().apply(config);

    m_pivotMotor.setPosition(0);
  }

  public void setOutput(double output) {
    double clamped =
        MathUtil.clamp(output, -ShooterPivotConstants.MAX_OUTPUT, ShooterPivotConstants.MAX_OUTPUT);
    m_pivotMotor.setControl(m_dutyCycleRequest.withOutput(clamped));
  }

  public void stop() {
    m_pivotMotor.setControl(m_neutralRequest);
  }

  public double getPosition() {
    return m_pivotMotor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return m_pivotMotor.getVelocity().getValueAsDouble();
  }

  public void zeroEncoder() {
    m_pivotMotor.setPosition(0);
  }

  public Command manualControlCommand(DoubleSupplier axisSupplier) {
    return run(() -> {
          double raw = axisSupplier.getAsDouble();
          double deadbanded = MathUtil.applyDeadband(raw, ShooterPivotConstants.DEADBAND);
          setOutput(deadbanded * ShooterPivotConstants.MAX_OUTPUT);
        })
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot Manual");
  }

  public Command zeroEncoderCommand() {
    return runOnce(this::zeroEncoder).withName("ShooterPivot Zero Encoder");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterPivot/Position (rot)", getPosition());
    SmartDashboard.putNumber("ShooterPivot/Velocity (rps)", getVelocity());
    SmartDashboard.putNumber(
        "ShooterPivot/SupplyCurrent", m_pivotMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "ShooterPivot/StatorCurrent", m_pivotMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "ShooterPivot/MotorVoltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "ShooterPivot/DutyCycle", m_pivotMotor.getDutyCycle().getValueAsDouble());
  }
}
