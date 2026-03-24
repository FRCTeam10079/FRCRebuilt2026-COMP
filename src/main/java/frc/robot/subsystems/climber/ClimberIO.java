package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  class ClimberIOInputs {}

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {}
}
