package frc.robot.constants;

public class ClimberConstants {

  // ==================== HARDWARE IDs ====================
  public static final int CLIMBER_MOTOR_ID = 26;
  public static final String CLIMBER_CANBUS = "rio";

  // ==================== MOTOR CONFIGURATION ====================
  /** Set true if motor positive direction needs to be inverted for extend. */
  public static final boolean MOTOR_INVERTED = true;

  /** Enable TalonFX FOC for smoother voltage control under heavy climb load. */
  public static final boolean ENABLE_FOC = true;

  public static final int SUPPLY_CURRENT_LIMIT = 30;
  public static final int STATOR_CURRENT_LIMIT = 40;

  /** Frequency (Hz) for climber Talon status signals. */
  public static final double STATUS_SIGNAL_UPDATE_HZ = 50.0;

  /** Debounce time before declaring motor connected/disconnected. */
  public static final double MOTOR_CONNECTED_DEBOUNCE_SECONDS = 0.25;

  // ==================== MECHANISM GEOMETRY ====================
  /**
   * AndyMark CIAB recommendation for Kraken X60 is typically a 12:1 Sport ratio.
   *
   * <p>Gonna keep using GEAR_RATIO but adding this to log.
   */
  public static final double CIAB_RECOMMENDED_KRAKEN_RATIO = 12.0;

  /** Installed climber reduction (motor turns : winch turns). */
  public static final double GEAR_RATIO = 25.0;

  /** CIAB stage travel from AndyMark docs (inches per stage). */
  public static final double CIAB_STAGE_TRAVEL_INCHES = 24.5;

  /** Set to 1 for single-stage CIAB, 2 for two-stage CIAB. (For future...) */
  public static final int CIAB_STAGE_COUNT = 1;

  /** Spool diameter in inches. */
  public static final double SPOOL_DIAMETER_INCHES = 0.541;

  /** Spool circumference in inches (linear distance per spool revolution). */
  public static final double SPOOL_CIRCUMFERENCE_INCHES = Math.PI * SPOOL_DIAMETER_INCHES;

  /** Motor rotations required per inch of linear travel. */
  public static final double MOTOR_ROTS_PER_INCH = GEAR_RATIO / SPOOL_CIRCUMFERENCE_INCHES;

  /** Full mechanism travel in inches. */
  public static final double FULL_TRAVEL_INCHES = CIAB_STAGE_COUNT * CIAB_STAGE_TRAVEL_INCHES;

  // ==================== POSITION THRESHOLDS (motor rotations)
  // ====================
  /** Motor rotations at full retract (encoder zero reference). */
  public static final double FULL_RETRACT_ROTATIONS = 0.0;

  /** Motor rotations at full extension based on CIAB travel and winch geometry. */
  public static final double FULL_EXTEND_ROTATIONS = FULL_TRAVEL_INCHES * MOTOR_ROTS_PER_INCH;

  /** Motor rotations at which the climb is scored (minimum L1). */
  public static final double CLIMB_SCORED_ROTATIONS = 88.0;

  /** Motor rotations target for autonomous climb (prioritize speed over full travel). */
  public static final double AUTO_CLIMB_TARGET_ROTATIONS = 88.0;

  /** Tolerance band for "at position" checks (motor rotations). */
  public static final double POSITION_TOLERANCE_ROTATIONS = 5.0;

  // ==================== VOLTAGE COMMANDS ====================
  /** Voltage to pay out rope (positive = extend direction). */
  public static final double EXTEND_VOLTAGE = 4.0;

  /** Voltage to wind rope in / lift robot (negative = retract direction). */
  public static final double RETRACT_VOLTAGE = -5.0;

  protected ClimberConstants() {}
}
