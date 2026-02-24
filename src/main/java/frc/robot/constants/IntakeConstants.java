package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;

public class IntakeConstants {
  public static class Pivot {
    public static final int MOTOR_ID = 24;

    public static final Angle INTAKE_POSITION = Rotations.of(0);
    public static final Angle STOWED_POSITION = Rotations.of(-6.25);

    public static final int SUPPLY_CURRENT_LIMIT = 40;
    public static final int STATOR_CURRENT_LIMIT = 80;

    /** Stator current (amps) above which the pivot is considered stalling. */
    public static final double STALL_CURRENT_THRESHOLD = 35.0;
    /** How long (seconds) current must exceed the threshold before declaring a stall. */
    public static final Time STALL_TIME_THRESHOLD = Seconds.of(0.25);

    public static final Angle DEPLOY_TOLERANCE = Rotations.of(0.05);

    public static final double KA = 0;
    public static final double KS = 0.4;
    public static final double KG = 0;
    public static final double KP = 1.8;
    public static final double KI = 0;
    public static final double KD = 0.25;
    public static final double KV = 0;

    protected Pivot() {}
  }

  public static class Wheels {
    public static final int MOTOR_ID = 19;

    public static final int SUPPLY_CURRENT_LIMIT = 40;
    public static final int STATOR_CURRENT_LIMIT = 80;

    public static final double INTAKE_IN_RPM = 2600;
    public static final double INTAKE_OUT_RPM = -2200;

    public static final double KA = 0;
    public static final double KS = 0.1;
    public static final double KP = 1;
    public static final double KI = 0;
    public static final double KD = 0.1;
    public static final double KV = 0.5;

    protected Wheels() {}
  }

  protected IntakeConstants() {}
}
