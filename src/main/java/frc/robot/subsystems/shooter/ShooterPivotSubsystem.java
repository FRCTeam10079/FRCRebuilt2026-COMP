// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterPivotConstants;
import java.util.function.DoubleSupplier;

public class ShooterPivotSubsystem extends SubsystemBase {

  private final TalonFX m_pivotMotor;

  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  private boolean m_isHomed = true;
  private double m_targetAngleDegrees = ShooterPivotConstants.MIN_ANGLE_DEGREES;

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

    config.Slot0 = new Slot0Configs()
        .withKP(ShooterPivotConstants.KP)
        .withKI(ShooterPivotConstants.KI)
        .withKD(ShooterPivotConstants.KD)
        .withKS(ShooterPivotConstants.KS)
        .withKV(ShooterPivotConstants.KV)
        .withKG(ShooterPivotConstants.KG)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    config.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(ShooterPivotConstants.MOTION_MAGIC_CRUISE_VELOCITY)
        .withMotionMagicAcceleration(ShooterPivotConstants.MOTION_MAGIC_ACCELERATION)
        .withMotionMagicJerk(ShooterPivotConstants.MOTION_MAGIC_JERK);

    config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(
            ShooterPivotConstants.degreesToMotorRotations(
                ShooterPivotConstants.MAX_ANGLE_DEGREES - ShooterPivotConstants.MIN_ANGLE_DEGREES))
        .withReverseSoftLimitThreshold(0.0);

    m_pivotMotor.getConfigurator().apply(config);

    m_pivotMotor.setPosition(0);
  }

  public void setAngle(double angleDegrees) {
    angleDegrees = MathUtil.clamp(
        angleDegrees,
        ShooterPivotConstants.MIN_ANGLE_DEGREES,
        ShooterPivotConstants.MAX_ANGLE_DEGREES);
    m_targetAngleDegrees = angleDegrees;

    double motorRotations = ShooterPivotConstants.degreesToMotorRotations(
        angleDegrees - ShooterPivotConstants.MIN_ANGLE_DEGREES);
    m_pivotMotor.setControl(m_motionMagicRequest.withPosition(motorRotations));
  }

  public double getCurrentAngleDegrees() {
    return ShooterPivotConstants.motorRotationsToDegrees(
        m_pivotMotor.getPosition().getValueAsDouble())
        + ShooterPivotConstants.MIN_ANGLE_DEGREES;
  }

  public boolean isAtAngle(double targetDegrees, double toleranceDegrees) {
    return Math.abs(getCurrentAngleDegrees() - targetDegrees) <= toleranceDegrees;
  }

  public boolean isAtTarget() {
    return isAtAngle(m_targetAngleDegrees, ShooterPivotConstants.SHOOTING_TOLERANCE_DEGREES);
  }

  public boolean isHomed() {
    return m_isHomed;
  }

  public double getTargetAngleDegrees() {
    return m_targetAngleDegrees;
  }

  public void setOutput(double output) {
    double clamped = MathUtil.clamp(
        output, -ShooterPivotConstants.MANUAL_MAX_OUTPUT, ShooterPivotConstants.MANUAL_MAX_OUTPUT);
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

  public void reZeroIfNeeded() {
    if (m_pivotMotor.getPosition().getValueAsDouble() < 0.0) {
      m_pivotMotor.setPosition(0);
    }
  }

  private void enableSoftwareLimits() {
    var softLimits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(
            ShooterPivotConstants.degreesToMotorRotations(
                ShooterPivotConstants.MAX_ANGLE_DEGREES - ShooterPivotConstants.MIN_ANGLE_DEGREES))
        .withReverseSoftLimitThreshold(0.0);
    m_pivotMotor.getConfigurator().apply(softLimits);
  }

  public Command homeCommand() {
    final int[] stallCounter = { 0 };

    return Commands.sequence(
        Commands.runOnce(() -> {
          m_isHomed = false;
          stallCounter[0] = 0;
        }),
        run(() -> {
          m_pivotMotor.setControl(
              m_dutyCycleRequest.withOutput(ShooterPivotConstants.HOMING_SPEED));

          double statorCurrent = m_pivotMotor.getStatorCurrent().getValueAsDouble();
          if (statorCurrent > ShooterPivotConstants.HOMING_CURRENT_THRESHOLD) {
            stallCounter[0]++;
          } else {
            stallCounter[0] = 0;
          }
        }).until(() -> stallCounter[0] >= ShooterPivotConstants.HOMING_STALL_CYCLES),
        Commands.runOnce(() -> {
          m_pivotMotor.setPosition(0);
          m_isHomed = true;
          enableSoftwareLimits();
          m_targetAngleDegrees = ShooterPivotConstants.MIN_ANGLE_DEGREES;
        }),
        Commands.runOnce(this::stop))
        .withName("ShooterPivot Home");
  }

  public Command trackAngleCommand(DoubleSupplier angleSupplier) {
    return run(() -> setAngle(angleSupplier.getAsDouble()))
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot Track Angle");
  }

  public Command goToAngleCommand(double angleDegrees) {
    return run(() -> setAngle(angleDegrees))
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot GoTo " + angleDegrees + "deg");
  }

  public Command manualControlCommand(DoubleSupplier axisSupplier) {
    return run(() -> {
      double raw = axisSupplier.getAsDouble();
      double deadbanded = MathUtil.applyDeadband(raw, ShooterPivotConstants.MANUAL_DEADBAND);
      setOutput(deadbanded * ShooterPivotConstants.MANUAL_MAX_OUTPUT);
    })
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot Manual");
  }

  public Command zeroEncoderCommand() {
    return runOnce(() -> m_pivotMotor.setPosition(0)).withName("ShooterPivot Zero Encoder");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterPivot/AngleDegrees", getCurrentAngleDegrees());
    SmartDashboard.putNumber("ShooterPivot/TargetAngleDegrees", m_targetAngleDegrees);
    SmartDashboard.putNumber("ShooterPivot/Position (rot)", getPosition());
    SmartDashboard.putNumber("ShooterPivot/Velocity (rps)", getVelocity());
    SmartDashboard.putBoolean("ShooterPivot/IsHomed", m_isHomed);
    SmartDashboard.putBoolean("ShooterPivot/AtTarget", isAtTarget());
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
