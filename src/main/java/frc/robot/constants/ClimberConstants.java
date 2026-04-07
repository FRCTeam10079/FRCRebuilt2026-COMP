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

  /**
   * Peak command voltages allowed by TalonFX voltage requests. Reduced to 2V to prevent the
   * string/belay from breaking under excessive commanded voltage.
   */
  public static final double PEAK_FORWARD_VOLTAGE = 2.0;

  public static final double PEAK_REVERSE_VOLTAGE = -2.0;

  public static final int SUPPLY_CURRENT_LIMIT = 30;
  public static final int STATOR_CURRENT_LIMIT = 60;

  // ==================== VOLTAGE CONTROL ====================
  /** Voltage applied when extending the climber (positive = extend direction). */
  public static final double EXTEND_VOLTAGE = 12.0;

  /** Voltage applied when retracting the climber (negative = retract direction). */
  public static final double RETRACT_VOLTAGE = -12.0;

  // ==================== STALL DETECTION ====================
  /**
   * Stator current (amps) above this threshold indicates the motor is under heavy load or stalled.
   * Normal running current is ~5-9A; stalled against a hard stop will hit the 60A stator limit.
   */
  public static final double STALL_CURRENT_THRESHOLD_AMPS = 30.0;

  /**
   * Velocity (RPS) below this threshold indicates the motor is barely moving. Normal running
   * velocity is 30-80 RPS; at a stall it drops to ~0 RPS.
   */
  public static final double STALL_VELOCITY_THRESHOLD_RPS = 5.0;

  /**
   * Stall condition must persist this many seconds before declaring done. Prevents false triggers
   * from brief current spikes.
   */
  public static final double STALL_DEBOUNCE_SECONDS = 0.25;

  /**
   * Ignore stall detection for this many seconds after starting to move. The motor draws high
   * current during initial acceleration which would look like a stall.
   */
  public static final double RAMP_UP_SECONDS = 0.5;

  // ==================== STATUS SIGNALS ====================
  /** Frequency (Hz) for climber Talon status signals. */
  public static final double STATUS_SIGNAL_UPDATE_HZ = 50.0;

  /** Debounce time before declaring motor connected/disconnected. */
  public static final double MOTOR_CONNECTED_DEBOUNCE_SECONDS = 0.25;

  // ==================== MECHANISM GEOMETRY ====================
  /** Installed climber reduction (motor turns : winch turns). */
  public static final double GEAR_RATIO = 25.0;

  protected ClimberConstants() {}
}
