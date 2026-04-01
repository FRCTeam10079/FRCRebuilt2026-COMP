package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  class ClimberIOInputs {
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double deviceTempCelsius = 0.0;
    public int faultField = 0;
    public int stickyFaultField = 0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {}
}
