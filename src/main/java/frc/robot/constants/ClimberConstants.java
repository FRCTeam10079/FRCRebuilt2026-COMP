package frc.robot.constants;

public class ClimberConstants {

  // ==================== HARDWARE IDs ====================
  public static final int CLIMBER_MOTOR_ID = 26;
  public static final String CLIMBER_CANBUS = "rio";

  // ==================== MOTOR CONFIGURATION ====================
  /** Set true if motor positive direction needs to be inverted for extend. */
  public static final boolean MOTOR_INVERTED = true;

  /** Enable TalonFX FOC for smoother control under heavy climb load. */
  public static final boolean ENABLE_FOC = true;

  /** Peak command voltages allowed by TalonFX closed-loop requests. */
  public static final double PEAK_FORWARD_VOLTAGE = 12.0;

  public static final double PEAK_REVERSE_VOLTAGE = -12.0;

  public static final int SUPPLY_CURRENT_LIMIT = 30;
  public static final int STATOR_CURRENT_LIMIT = 60;

  // ==================== CLOSED-LOOP TUNING ====================
  /** Mechanism velocity gain estimate in rotations/sec per volt. */
  public static final double RPS_PER_VOLT = 7.9;

  public static final double KP = 4.0;
  public static final double KI = 0.0;
  public static final double KD = 0.1;
  public static final double KS = 0.25;
  public static final double KV = 1.0 / RPS_PER_VOLT;
  public static final double KA = 0.01;
  public static final double KG = 0.0;

  /** Motion Magic trajectory limits — kept conservative for climb load. */
  public static final double MOTION_MAGIC_CRUISE_VELOCITY_RPS = 80.0;

  public static final double MOTION_MAGIC_ACCELERATION_RPS2 = 40.0;

  /** Jerk limit for smoother Motion Magic S-curve. 0 = trapezoidal (no jerk limit). */
  public static final double MOTION_MAGIC_JERK_RPS3 = 800.0;

  /** Frequency (Hz) for climber Talon status signals. */
  public static final double STATUS_SIGNAL_UPDATE_HZ = 50.0;

  /** Debounce time before declaring motor connected/disconnected. */
  public static final double MOTOR_CONNECTED_DEBOUNCE_SECONDS = 0.25;

  // ==================== MECHANISM GEOMETRY ====================
  /** Installed climber reduction (motor turns : winch turns). */
  public static final double GEAR_RATIO = 25.0;

  // ==================== POSITION THRESHOLDS (motor rotations)
  // ====================
  /** Motor rotations at full retract (encoder zero reference). */
  public static final double FULL_RETRACT_ROTATIONS = 0.0;

  /** Motor rotations at full extension (physical max ~164, according to my test). */
  public static final double FULL_EXTEND_ROTATIONS = 164.0;

  /** Motor rotations for the climb (retract) position - halfway, tunable. */
  public static final double CLIMB_RETRACT_ROTATIONS = 82.0;

  /** Tolerance band for "at position" checks (motor rotations). */
  public static final double POSITION_TOLERANCE_ROTATIONS = 5.0;

  /** Sim-only position controller proportional gain (volts/rotation). */
  public static final double SIM_POSITION_KP_VOLTS_PER_ROT = 0.08;

  protected ClimberConstants() {}
}
