package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ClimberConstants;

/**
 * Simulated climber IO. Models a first-order winch driven by voltage, tracking motor position and
 * velocity.
 */
public class ClimberIOSim implements ClimberIO {
  private static final double LOOP_PERIOD_SEC = 0.02;
  private static final double NOMINAL_VOLTAGE = 12.0;

  /** Approximate free speed of the Kraken X60 in RPS (~6000 RPM / 60). */
  private static final double KV_RPS_PER_VOLT = (6000.0 / 60.0) / NOMINAL_VOLTAGE;

  /** Time constant for velocity ramping (seconds). Models rotor + spool inertia. */
  private static final double TAU = 0.3;

  private enum ControlMode {
    NEUTRAL,
    VOLTAGE,
    POSITION
  }

  private ControlMode controlMode = ControlMode.NEUTRAL;

  private double positionRotations = 0.0;
  private double velocityRPS = 0.0;
  private double appliedVolts = 0.0;
  private double positionSetpointRotations = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorConnected = true;

    if (controlMode == ControlMode.POSITION) {
      double error = positionSetpointRotations - positionRotations;
      appliedVolts = MathUtil.clamp(
          error * ClimberConstants.SIM_POSITION_KP_VOLTS_PER_ROT,
          ClimberConstants.PEAK_REVERSE_VOLTAGE,
          ClimberConstants.PEAK_FORWARD_VOLTAGE);
    } else if (controlMode == ControlMode.NEUTRAL) {
      appliedVolts = 0.0;
    }

    // First-order model: velocity approaches target exponentially
    double targetVelocityRPS = appliedVolts * KV_RPS_PER_VOLT;
    if (controlMode == ControlMode.NEUTRAL) {
      targetVelocityRPS = 0.0;
    }
    double alpha = 1.0 - Math.exp(-LOOP_PERIOD_SEC / TAU);
    velocityRPS += alpha * (targetVelocityRPS - velocityRPS);

    // Integrate position
    positionRotations += velocityRPS * LOOP_PERIOD_SEC;

    // Clamp position to software limits (simulates TalonFX software limits)
    positionRotations = MathUtil.clamp(
        positionRotations,
        ClimberConstants.FULL_RETRACT_ROTATIONS,
        ClimberConstants.FULL_EXTEND_ROTATIONS);

    // If clamped at a limit, zero velocity in that direction
    if (positionRotations <= ClimberConstants.FULL_RETRACT_ROTATIONS && velocityRPS < 0) {
      velocityRPS = 0.0;
    }
    if (positionRotations >= ClimberConstants.FULL_EXTEND_ROTATIONS && velocityRPS > 0) {
      velocityRPS = 0.0;
    }

    inputs.positionRotations = positionRotations;
    inputs.velocityRPS = velocityRPS;
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(appliedVolts) * 3.0; // rough estimate
    inputs.statorCurrentAmps = Math.abs(appliedVolts) * 5.0;
    inputs.tempCelsius = 30.0; // nominal sim temperature
    inputs.closedLoopError = (controlMode == ControlMode.POSITION)
        ? (positionSetpointRotations - positionRotations)
        : 0.0;
    inputs.closedLoopReference =
        (controlMode == ControlMode.POSITION) ? positionSetpointRotations : positionRotations;
    inputs.dutyCycle = appliedVolts / NOMINAL_VOLTAGE;
    inputs.supplyVoltage = NOMINAL_VOLTAGE;
  }

  @Override
  public void setVoltage(double volts) {
    controlMode = ControlMode.VOLTAGE;
    appliedVolts = MathUtil.clamp(volts, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
  }

  @Override
  public void setPosition(double rotations) {
    controlMode = ControlMode.POSITION;
    positionSetpointRotations = MathUtil.clamp(
        rotations, ClimberConstants.FULL_RETRACT_ROTATIONS, ClimberConstants.FULL_EXTEND_ROTATIONS);
  }

  @Override
  public void stop() {
    controlMode = ControlMode.NEUTRAL;
    appliedVolts = 0.0;
  }

  @Override
  public void setEncoderPosition(double rotations) {
    positionRotations = rotations;
    positionSetpointRotations = rotations;
  }
}
