package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX motor;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public ClimberIOTalonFX() {
    motor =
        new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID, new CANBus(ClimberConstants.CLIMBER_CANBUS));
    configureMotor();

    // Zero the encoder at construction time.
    // Assumption: mechanism is fully retracted when the robot powers on.
    motor.setPosition(0.0);
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Brake mode is critical - holds robot weight when motor is neutral
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = ClimberConstants.MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    // Current limits to protect the motor under sustained climbing load
    config.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(ClimberConstants.SUPPLY_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(ClimberConstants.STATOR_CURRENT_LIMIT);

    // Software limits to prevent over-extension and over-retraction
    config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ClimberConstants.FULL_EXTEND_ROTATIONS)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ClimberConstants.FULL_RETRACT_ROTATIONS);

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.positionRotations = motor.getPosition().getValueAsDouble();
    inputs.velocityRPS = motor.getVelocity().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }

  @Override
  public void setEncoderPosition(double rotations) {
    motor.setPosition(rotations);
  }
}
