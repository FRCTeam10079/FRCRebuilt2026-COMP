package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX masterMotor;
  private final TalonFX slaveMotor;
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final Follower followerRequest;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ShooterIOTalonFX() {
    masterMotor = new TalonFX(ShooterConstants.MASTER_MOTOR_ID);
    slaveMotor = new TalonFX(ShooterConstants.SLAVE_MOTOR_ID);
    followerRequest = new Follower(ShooterConstants.MASTER_MOTOR_ID, MotorAlignmentValue.Aligned);
    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    masterConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(70)
        .withSupplyCurrentLowerLimit(40)
        .withSupplyCurrentLowerTime(1.0)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(120);
    masterConfig.Slot0 = new Slot0Configs()
        .withKS(ShooterConstants.SHOOTER_KS)
        .withKV(ShooterConstants.SHOOTER_KV)
        .withKP(ShooterConstants.SHOOTER_KP)
        .withKI(ShooterConstants.SHOOTER_KI)
        .withKD(ShooterConstants.SHOOTER_KD);
    masterMotor.getConfigurator().apply(masterConfig);

    TalonFXConfiguration slaveConfig = new TalonFXConfiguration();
    slaveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    slaveConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(80);
    slaveMotor.getConfigurator().apply(slaveConfig);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.masterVelocityRPS = masterMotor.getVelocity().getValueAsDouble();
    inputs.masterSupplyCurrentAmps = masterMotor.getSupplyCurrent().getValueAsDouble();
    inputs.masterStatorCurrentAmps = masterMotor.getStatorCurrent().getValueAsDouble();
    inputs.masterVoltageVolts = masterMotor.getMotorVoltage().getValueAsDouble();
    inputs.slaveSupplyCurrentAmps = slaveMotor.getSupplyCurrent().getValueAsDouble();
    inputs.slaveStatorCurrentAmps = slaveMotor.getStatorCurrent().getValueAsDouble();
    inputs.slaveVoltageVolts = slaveMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    masterMotor.setControl(velocityRequest.withVelocity(rps));
    slaveMotor.setControl(followerRequest);
  }

  @Override
  public void setVoltage(double volts) {
    masterMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    masterMotor.setControl(neutralRequest);
    slaveMotor.setControl(neutralRequest);
  }
}
