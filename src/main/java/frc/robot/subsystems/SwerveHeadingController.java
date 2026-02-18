// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HeadingControllerConstants;

/**
 * This class controls the rotational heading of the drivetrain seperately from translation control.
 *
 * <p>Some features I added: 1. State Machine: SNAP (high gains to quickly reach target) vs MAINTAIN
 * (low gains to hold) 2. Automatic state transitions: After snapping to target, automatically
 * switches to maintain 3. Separate from drivetrain: Can be used with any translation source (driver
 * input, auto path)
 *
 * <p>Usage: - Call setGoal() to set target heading in degrees (field-relative) - Call update() each
 * loop with current heading to get rotation output - The output is a normalized value (-1 to 1)
 * representing rotational velocity demand
 */
public class SwerveHeadingController {

  // ==================== STATE MACHINE ====================
  public enum HeadingControllerState {
    OFF, // No heading control - return 0 rotation
    SNAP, // Actively snapping to a target heading (higher gains)
    MAINTAIN // Holding current heading with lower gains
  }

  private HeadingControllerState m_state = HeadingControllerState.OFF;

  private final PIDController m_pidController;

  private double m_goalDegrees = 0.0;
  private double m_lastOutputTimestamp = 0.0;

  /** Creates a new SwerveHeadingController */
  public SwerveHeadingController() {
    // Initialize PID with SNAP gains (will be updated based on state)
    m_pidController = new PIDController(
        HeadingControllerConstants.SNAP_KP,
        HeadingControllerConstants.SNAP_KI,
        HeadingControllerConstants.SNAP_KD);

    // Enable continuous input for angle wrapping (-180 to 180)
    m_pidController.enableContinuousInput(-180, 180);

    // Set tolerance for "at goal" detection
    m_pidController.setTolerance(HeadingControllerConstants.HEADING_TOLERANCE_DEGREES);
  }

  // ==================== CONFIGURATION METHODS ====================

  /**
   * Set the heading controller state
   *
   * @param state The desired state (OFF, SNAP, or MAINTAIN)
   */
  public void setHeadingControllerState(HeadingControllerState state) {
    if (m_state != state) {
      m_state = state;
      updateGains();
    }
  }

  /** Get the current heading controller state */
  public HeadingControllerState getHeadingControllerState() {
    return m_state;
  }

  /**
   * Set the target heading goal
   *
   * @param goalDegrees Target heading in degrees (field-relative, -180 to 180)
   */
  public void setGoal(double goalDegrees) {
    // Normalize to -180 to 180 range
    m_goalDegrees = MathUtil.inputModulus(goalDegrees, -180, 180);
    m_pidController.setSetpoint(m_goalDegrees);
  }

  /** Get the current heading goal */
  public double getGoal() {
    return m_goalDegrees;
  }

  // ==================== UPDATE LOOP ====================

  /**
   * Update the heading controller and calculate rotation output.
   *
   * <p>This should be called every robot loop with the current heading. The output is a normalized
   * rotation demand (-1 to 1) that should be multiplied by max angular velocity before applying to
   * the drivetrain.
   *
   * @param currentHeadingDegrees Current robot heading in degrees (field-relative)
   * @return Rotation output (-1 to 1), where positive is counter-clockwise
   */
  public double update(double currentHeadingDegrees) {
    // Normalize input to -180 to 180 range
    currentHeadingDegrees = MathUtil.inputModulus(currentHeadingDegrees, -180, 180);

    // Handle state machine
    switch (m_state) {
      case OFF:
        return 0.0;

      case SNAP:
        // Calculate output with high gains
        double snapOutput = m_pidController.calculate(currentHeadingDegrees);

        // Check if we've reached the goal - if so, transition to MAINTAIN
        if (isAtGoal()) {
          setHeadingControllerState(HeadingControllerState.MAINTAIN);
        }

        // Clamp output to valid range
        return MathUtil.clamp(snapOutput, -1.0, 1.0);

      case MAINTAIN:
        // Calculate output with lower gains
        double maintainOutput = m_pidController.calculate(currentHeadingDegrees);

        // Clamp output to valid range
        return MathUtil.clamp(maintainOutput, -1.0, 1.0);

      default:
        return 0.0;
    }
  }

  /**
   * Check if the heading controller is at the goal within tolerance
   *
   * @return true if current heading is within tolerance of goal
   */
  public boolean isAtGoal() {
    return m_pidController.atSetpoint();
  }

  /**
   * Get the current heading error in degrees
   *
   * @return Position error in degrees (positive = need to rotate CCW)
   */
  public double getError() {
    return m_pidController.getError();
  }

  // HELPER METHODS

  /** Update PID gains based on current state */
  private void updateGains() {
    switch (m_state) {
      case SNAP:
        m_pidController.setPID(
            HeadingControllerConstants.SNAP_KP,
            HeadingControllerConstants.SNAP_KI,
            HeadingControllerConstants.SNAP_KD);
        break;

      case MAINTAIN:
        m_pidController.setPID(
            HeadingControllerConstants.MAINTAIN_KP,
            HeadingControllerConstants.MAINTAIN_KI,
            HeadingControllerConstants.MAINTAIN_KD);
        break;

      case OFF:
      default:
        // No gains needed when off
        break;
    }

    // Log gain changes for debugging
    SmartDashboard.putString("HeadingController/State", m_state.toString());
    SmartDashboard.putNumber("HeadingController/ActiveKP", m_pidController.getP());
  }

  /** Reset the heading controller Clears accumulated integral error and resets state */
  public void reset() {
    m_pidController.reset();
    m_state = HeadingControllerState.OFF;
    m_goalDegrees = 0.0;
  }

  /** Log telemetry data to SmartDashboard Call this from a subsystem's periodic() method */
  public void logTelemetry(double currentHeadingDegrees) {
    SmartDashboard.putString("HeadingController/State", m_state.toString());
    SmartDashboard.putNumber("HeadingController/GoalDeg", m_goalDegrees);
    SmartDashboard.putNumber("HeadingController/CurrentDeg", currentHeadingDegrees);
    SmartDashboard.putNumber("HeadingController/ErrorDeg", getError());
    SmartDashboard.putBoolean("HeadingController/AtGoal", isAtGoal());
  }
}
