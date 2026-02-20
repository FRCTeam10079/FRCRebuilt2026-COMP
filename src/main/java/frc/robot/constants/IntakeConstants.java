package frc.robot.constants;

public class IntakeConstants {
  public static class Pivot {
    public static final int MOTOR_ID = 24;

    public static final double INTAKE_POSITION = 0;
    public static final double STOWED_POSITION = -5.5;

    public static final int SUPPLY_CURRENT_LIMIT = 40;
    public static final int STATOR_CURRENT_LIMIT = 80;

    public static final double DEPLOY_TOLERANCE = 0.1;

    public static final double KA = 0;
    public static final double KS = 0.4;
    public static final double KG = 0;
    public static final double KP = 1.4;
    public static final double KI = 0.05;
    public static final double KD = 0.1;
    public static final double KV = 0;

    protected Pivot() {}
  }

  public static class Wheels {
    public static final int MOTOR_ID = 19;

    public static final int SUPPLY_CURRENT_LIMIT = 40;
    public static final int STATOR_CURRENT_LIMIT = 80;

    public static final double INTAKE_IN_RPM = 2000;
    public static final double INTAKE_OUT_RPM = -2000;

    public static final double KA = 0;
    public static final double KS = 0.1;
    public static final double KP = 0.3;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KV = 0.5;

    protected Wheels() {}
  }

  protected IntakeConstants() {}
}
