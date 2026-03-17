package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public double feederVelocityRPS = 0.0;
    public double spindexerVelocityRPS = 0.0;
    public double feederSupplyCurrentAmps = 0.0;
    public double feederStatorCurrentAmps = 0.0;
    public double feederVoltageVolts = 0.0;
    public double spindexerSupplyCurrentAmps = 0.0;
    public double spindexerStatorCurrentAmps = 0.0;
    public double spindexerVoltageVolts = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setFeederVelocity(double rps) {}

  default void setSpindexerVelocity(double rps) {}

  default void stop() {}
}
