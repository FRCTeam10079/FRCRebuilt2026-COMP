package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {

  private final TalonFX feederMotor;
  private final TalonFX spindexerMotor;
  private final VelocityVoltage feederRequest = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage spindexerRequest = new VelocityVoltage(0).withSlot(0);

  public IndexerIOTalonFX() {
    feederMotor = new TalonFX(IndexerConstants.kFeederMotorID, new CANBus("rio"));
    spindexerMotor = new TalonFX(IndexerConstants.kSpindexerMotorID, new CANBus("rio"));
    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit;
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.Slot0 = new Slot0Configs()
        .withKP(IndexerConstants.kFeederKP)
        .withKI(IndexerConstants.kFeederKI)
        .withKD(IndexerConstants.kFeederKD)
        .withKS(IndexerConstants.kFeederKS)
        .withKV(IndexerConstants.kFeederKV)
        .withKA(IndexerConstants.kFeederKA)
        .withKG(IndexerConstants.kFeederKG);
    feederMotor.getConfigurator().apply(feederConfig);

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    spindexerConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit;
    spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    spindexerConfig.Slot0 = new Slot0Configs()
        .withKP(IndexerConstants.kSpindexerKP)
        .withKI(IndexerConstants.kSpindexerKI)
        .withKD(IndexerConstants.kSpindexerKD)
        .withKS(IndexerConstants.kSpindexerKS)
        .withKV(IndexerConstants.kSpindexerKV)
        .withKA(IndexerConstants.kSpindexerKA)
        .withKG(IndexerConstants.kSpindexerKG);
    spindexerMotor.getConfigurator().apply(spindexerConfig);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.feederVelocityRPS = feederMotor.getVelocity().getValueAsDouble();
    inputs.spindexerVelocityRPS = spindexerMotor.getVelocity().getValueAsDouble();
    inputs.feederSupplyCurrentAmps = feederMotor.getSupplyCurrent().getValueAsDouble();
    inputs.feederStatorCurrentAmps = feederMotor.getStatorCurrent().getValueAsDouble();
    inputs.feederVoltageVolts = feederMotor.getMotorVoltage().getValueAsDouble();
    inputs.spindexerSupplyCurrentAmps = spindexerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.spindexerStatorCurrentAmps = spindexerMotor.getStatorCurrent().getValueAsDouble();
    inputs.spindexerVoltageVolts = spindexerMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setFeederVelocity(double rps) {
    feederMotor.setControl(feederRequest.withVelocity(rps));
  }

  @Override
  public void setSpindexerVelocity(double rps) {
    spindexerMotor.setControl(spindexerRequest.withVelocity(rps));
  }

  @Override
  public void stop() {
    feederMotor.stopMotor();
    spindexerMotor.stopMotor();
  }
}
