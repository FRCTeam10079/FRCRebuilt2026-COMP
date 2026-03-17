package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeWheelsIOTalonFX implements IntakeWheelsIO {

  private final TalonFX intakeMotor;
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  public IntakeWheelsIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.Wheels.MOTOR_ID, Constants.kCANBus);
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.withSupplyCurrentLimit(IntakeConstants.Wheels.SUPPLY_CURRENT_LIMIT);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.withStatorCurrentLimit(IntakeConstants.Wheels.STATOR_CURRENT_LIMIT);
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.withKA(IntakeConstants.Wheels.KA)
        .withKV(IntakeConstants.Wheels.KV)
        .withKD(IntakeConstants.Wheels.KD)
        .withKS(IntakeConstants.Wheels.KS)
        .withKI(IntakeConstants.Wheels.KI)
        .withKP(IntakeConstants.Wheels.KP);

    intakeMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeWheelsIOInputs inputs) {
    inputs.velocityRPS = intakeMotor.getVelocity().getValueAsDouble();
    inputs.supplyCurrentAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = intakeMotor.getStatorCurrent().getValueAsDouble();
    inputs.voltageVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    intakeMotor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void stop() {
    intakeMotor.setControl(neutralRequest);
  }
}
