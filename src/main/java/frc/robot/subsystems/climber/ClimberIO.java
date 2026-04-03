package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  class ClimberIOInputs {
    public boolean motorConnected = false;
    public double positionRotations = 0.0;
    public double velocityRPS = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double tempCelsius = 0.0;
    public int faultField = 0;
    public int stickyFaultField = 0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {}

  default void setEncoderPosition(double rotations) {}
}
