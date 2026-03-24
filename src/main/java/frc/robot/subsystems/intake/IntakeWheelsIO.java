package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWheelsIO {
  @AutoLog
  class IntakeWheelsIOInputs {
    public double velocityRPS = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double voltageVolts = 0.0;
  }

  default void updateInputs(IntakeWheelsIOInputs inputs) {}

  default void setVelocity(double rps) {}

  default void stop() {}
}
