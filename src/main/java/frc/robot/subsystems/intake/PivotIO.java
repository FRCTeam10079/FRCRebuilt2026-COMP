package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  class PivotIOInputs {
    public double positionRotations = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double voltageVolts = 0.0;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  /** Set the MotionMagic position target. */
  default void setMotionMagicPosition(Angle position) {}

  /** Set motor to NeutralOut (coast/brake hold). */
  default void setNeutral() {}

  /** Called every loop for NetworkedTalonFX periodic updates. */
  default void periodic() {}
}
