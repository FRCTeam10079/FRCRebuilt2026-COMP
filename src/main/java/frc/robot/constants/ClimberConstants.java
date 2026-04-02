package frc.robot.constants;

public class ClimberConstants {

  // ==================== HARDWARE IDs ====================
  public static final int CLIMBER_MOTOR_ID = 26;
  public static final String CLIMBER_CANBUS = "rio";

  // ==================== MOTOR CONFIGURATION ====================
  /** Set true if motor positive direction needs to be inverted for extend. */
  public static final boolean MOTOR_INVERTED = false;

  public static final int SUPPLY_CURRENT_LIMIT = 30;
  public static final int STATOR_CURRENT_LIMIT = 40;

  // ==================== MECHANISM GEOMETRY ====================
  /** AndyMark Climber-in-a-Box gear ratio (motor turns : output turns). */
  public static final double GEAR_RATIO = 25.0;

  /** Spool diameter in inches. */
  public static final double SPOOL_DIAMETER_INCHES = 0.541;

  /** Spool circumference in inches (linear distance per spool revolution). */
  public static final double SPOOL_CIRCUMFERENCE_INCHES = Math.PI * SPOOL_DIAMETER_INCHES;

  /** Motor rotations required per inch of linear travel. */
  public static final double MOTOR_ROTS_PER_INCH = GEAR_RATIO / SPOOL_CIRCUMFERENCE_INCHES;

  /** Full mechanism travel in inches. */
  public static final double FULL_TRAVEL_INCHES = 5.0;

  // ==================== POSITION THRESHOLDS (motor rotations)
  // ====================
  /** Motor rotations at full retract (encoder zero reference). */
  public static final double FULL_RETRACT_ROTATIONS = 0.0;

  /** Motor rotations at full extension (~24.5 inches of travel). */
  public static final double FULL_EXTEND_ROTATIONS = 360.0;

  /** Motor rotations at which the climb is scored (minimum L1). */
  public static final double CLIMB_SCORED_ROTATIONS = 88.0;

  /**
   * Motor rotations target for autonomous climb (prioritize speed over full
   * travel).
   */
  public static final double AUTO_CLIMB_TARGET_ROTATIONS = 88.0;

  /** Tolerance band for "at position" checks (motor rotations). */
  public static final double POSITION_TOLERANCE_ROTATIONS = 5.0;

  // ==================== VOLTAGE COMMANDS ====================
  /** Voltage to pay out rope (positive = extend direction). */
  public static final double EXTEND_VOLTAGE = 4.0;

  /** Voltage to wind rope in / lift robot (negative = retract direction). */
  public static final double RETRACT_VOLTAGE = -5.0;

  protected ClimberConstants() {
  }
}
