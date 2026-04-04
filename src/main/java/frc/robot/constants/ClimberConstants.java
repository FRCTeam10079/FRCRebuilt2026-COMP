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

  // ==================== VOLTAGE CONTROL ====================
  /** Voltage applied when extending the climber (positive = extend direction). */
  public static final double EXTEND_VOLTAGE = 12.0;

  /** Voltage applied when retracting the climber (negative = retract direction). */
  public static final double RETRACT_VOLTAGE = -12.0;

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
