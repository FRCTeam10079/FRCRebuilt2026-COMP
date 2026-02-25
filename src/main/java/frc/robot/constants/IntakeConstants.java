package frc.robot.constants;

public class IntakeConstants {
  public static class Pivot {
    public static final int MOTOR_ID = 24;

    public static final double INTAKE_POSITION = 0;
    public static final double STOWED_POSITION = -6.25;

    public static final int SUPPLY_CURRENT_LIMIT = 30;
    public static final int STATOR_CURRENT_LIMIT = 40;

    /** Stator current (amps) above which the pivot is considered stalling. */
    public static final double STALL_CURRENT_THRESHOLD = 30.0;
    /** How long (seconds) current must exceed the threshold before declaring a stall. */
    public static final double STALL_TIME_THRESHOLD = 0.25;

    public static final double DEPLOY_TOLERANCE = 0.05;

    /**
     * How long (seconds) the pivot must be at setpoint before switching to idle (NeutralOut). Brake
     * mode holds position mechanically once the motor is off.
     */
    public static final double IDLE_DEBOUNCE_SECONDS = 0.5;

    public static final double KA = 0;
    public static final double KS = 0.4;
    /**
     * Gravity compensation feedforward. Tune this by commanding the pivot to 90deg and measuring
     * the duty cycle needed to hold it still - that's approximately kG. With Arm_Cosine gravity
     * type, the controller applies kG * cos(angle) automatically. TODO: Tune this value on the
     * robot. Start at ~0.15 and adjust.
     */
    public static final double KG = 0.15;

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
