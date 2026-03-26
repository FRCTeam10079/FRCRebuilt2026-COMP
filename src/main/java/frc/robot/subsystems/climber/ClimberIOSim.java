package frc.robot.subsystems.climber;

/**
 * Simulated climber IO. The real climber is voltage-only with no position feedback, so the sim just
 * tracks whether voltage is being applied.
 */
public class ClimberIOSim implements ClimberIO {
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // ClimberIOInputs has no fields — nothing to populate
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
