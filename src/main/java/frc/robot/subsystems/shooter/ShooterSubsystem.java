// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;

/**
 * This subsystem contains: - Debounced "isReady()" check to ensure flywheel stability before
 * feeding - Single motor configuration (dual motor support commented out for future use)
 *
 * <p>Some features I added: 1. Use velocity control for consistent shot power regardless of battery
 * voltage 2. Implement a stability counter that requires the flywheel to be within tolerance for
 * multiple consecutive cycles before reporting "ready" 3. Coast mode when idle to prevent belt
 * skipping during rapid starts
 *
 * <p>NOTE: Dual motor (master/slave counter-rotating) code is commented out. Uncomment the slave
 * motor sections if we switch to a 2-motor setup.
 */
public class ShooterSubsystem extends SubsystemBase {

  // DUAL MOTOR: Uncomment for 2-motor setup
  private final TalonFX m_masterMotor = new TalonFX(ShooterConstants.MASTER_MOTOR_ID);
  // DUAL MOTOR: Uncomment for 2-motor setup
  private final TalonFX m_slaveMotor = new TalonFX(ShooterConstants.SLAVE_MOTOR_ID);

  private final VelocityVoltage m_velocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();
  // DUAL MOTOR: Uncomment for 2-motor setup
  private final Follower m_followerRequest;
  StatusSignal<AngularVelocity> velocitySignal = m_masterMotor.getVelocity();

  private AngularVelocity m_targetRPM = RPM.zero();
  private boolean m_isEnabled = false;

  // Stability tracking with debouncing
  // We require the flywheel to be within tolerance for STABILITY_CYCLES
  // consecutive
  // loops before we report "ready". This prevents false positives from RPM
  // flickering.
  private int m_stabilityCounter = 0;

  // Niche sysiD
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine m_sysIdRoutine;

  /** Creates a new ShooterSubsystem */
  public ShooterSubsystem() {
    m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> m_masterMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
            null,
            this));

    configureMotors();

    // DUAL MOTOR: Uncomment for 2-motor setup
    // Set up follower - slave follows master in opposite direction
    // This creates the counter-rotating flywheel configuration
    m_followerRequest = new Follower(ShooterConstants.MASTER_MOTOR_ID, MotorAlignmentValue.Aligned);
  }

  /** Configure both shooter motors with appropriate gains and limits */
  private void configureMotors() {
    // ==================== MASTER CONFIGURATION ====================
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();

    // Motor direction - NEEDS TO BE ADJUSTED BASED ON WHAT MECH PUTS
    masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast when neutral

    // Current limits
    // Stator current = torque output. Low stator limits = slow acceleration.
    // Phoenix 6 docs says 80A stator limit cuts acceleration by 56%! :O
    // We use a high stator on the flywheels (120A+) since no risk of wheel slip.
    // Supply current = battery draw. We need to limit this to prevent brownouts.
    masterConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(70) // Continuous supply limit (battery)
        .withSupplyCurrentLowerLimit(40) // Reduced limit after sustained draw
        .withSupplyCurrentLowerTime(1.0) // Time at limit before reducing
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(120); // High stator = max torque/accel!

    // Velocity PID gains (Slot 0)
    // THESE VALUES NEED TO BE TUNED FOR THE ROBOT
    // kV: Volts per RPS to maintain velocity (main feedforward term)
    // kS: Volts to overcome static friction
    // kP: Volts per RPS of error (feedback term)
    masterConfig.Slot0 = new Slot0Configs()
        .withKS(ShooterConstants.SHOOTER_KS)
        .withKV(ShooterConstants.SHOOTER_KV)
        .withKP(ShooterConstants.SHOOTER_KP)
        .withKI(ShooterConstants.SHOOTER_KI)
        .withKD(ShooterConstants.SHOOTER_KD);

    // Apply configuration
    m_masterMotor.getConfigurator().apply(masterConfig);

    // DUAL MOTOR: Uncomment for 2-motor setup
    // ==================== SLAVE CONFIGURATION ====================
    TalonFXConfiguration slaveConfig = new TalonFXConfiguration();
    slaveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //
    // // Current limits for slave (same as master)
    slaveConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(80);
    //
    m_slaveMotor.getConfigurator().apply(slaveConfig);
  }

  @Override
  public void periodic() {
    // Read current velocity from motor
    AngularVelocity currentRPM = m_masterMotor.getVelocity().getValue();

    // Update stability counter (debouncing logic)
    boolean isTargetPositive = m_targetRPM.gt(RPM.zero());
    if (m_isEnabled && isTargetPositive) {
      if (currentRPM.isNear(m_targetRPM, ShooterConstants.SHOOTER_SPEED_TOLERANCE)) {
        // Within tolerance - increment counter (capped at required cycles)
        m_stabilityCounter =
            Math.min(m_stabilityCounter + 1, ShooterConstants.STABILITY_CYCLES_REQUIRED);
      } else {
        // Outside tolerance - reset counter
        m_stabilityCounter = 0;
      }
    } else {
      // Shooter disabled - reset counter
      m_stabilityCounter = 0;
    }

    // Apply control to motors
    if (m_isEnabled && isTargetPositive) {
      // Convert RPM to RPS for Phoenix 6
      m_masterMotor.setControl(m_velocityRequest.withVelocity(m_targetRPM));
      // DUAL MOTOR: Uncomment for 2-motor setup
      m_slaveMotor.setControl(m_followerRequest);
    } else {
      // Coast when disabled
      m_masterMotor.setControl(m_neutralRequest);
      // DUAL MOTOR: Uncomment for 2-motor setup
      m_slaveMotor.setControl(m_neutralRequest);
    }

    // Telemetry
    SmartDashboard.putNumber("Shooter/TargetRPM", m_targetRPM.in(RPM));
    SmartDashboard.putNumber("Shooter/CurrentRPM", currentRPM.in(RPM));
    SmartDashboard.putNumber("Shooter/ErrorRPM", m_targetRPM.minus(currentRPM).in(RPM));
    SmartDashboard.putBoolean("Shooter/IsEnabled", m_isEnabled);
    SmartDashboard.putBoolean("Shooter/IsReady", isReady());
    SmartDashboard.putNumber("Shooter/StabilityCounter", m_stabilityCounter);
    // Current monitoring - When testing, make sure to watch these to verify limits
    // aren't clamping
    // output
    SmartDashboard.putNumber(
        "Shooter/SupplyCurrent", m_masterMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/StatorCurrent", m_masterMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/MotorVoltage", m_masterMotor.getMotorVoltage().getValueAsDouble());
  }

  /**
   * Set the target RPM for the shooter flywheels
   *
   * @param rpm Target velocity in rotations per minute
   */
  private void setTargetRPM(AngularVelocity rpm) {
    AngularVelocity clampedRPM =
        Constants.clamp(rpm, RPM.zero(), ShooterConstants.SHOOTER_MAX_SPEED);

    // Reset stability counter when setpoint changes significantly
    if (!clampedRPM.isNear(m_targetRPM, ShooterConstants.SHOOTER_SPEED_TOLERANCE)) {
      m_stabilityCounter = 0;
    }

    m_targetRPM = clampedRPM;
    m_isEnabled = clampedRPM.gt(RPM.zero());
  }

  /** Enable the shooter at the pre-configured spin-up RPM */
  private void spinUp() {
    setTargetRPM(ShooterConstants.SHOOTER_SPINUP_SPEED);
  }

  /** Stop the shooter (coast to stop) */
  public void stop() {
    m_targetRPM = RPM.zero();
    m_isEnabled = false;
    m_stabilityCounter = 0;
  }

  /**
   * Check if the shooter is ready to fire.
   *
   * <p>It does NOT just check if RPM is within tolerance - it requires the flywheel to have been
   * stable for multiple consecutive cycles.
   *
   * <p>This prevents false positives where RPM momentarily crosses the threshold but isn't actually
   * stable yet.
   *
   * @return true if shooter is spun up AND has been stable for sufficient time
   */
  public boolean isReady() {
    return m_isEnabled
        && m_targetRPM.gt(RPM.zero())
        && m_stabilityCounter >= ShooterConstants.STABILITY_CYCLES_REQUIRED;
  }

  /**
   * Check if the shooter is within tolerance (instantaneous check, no debounce) Useful for
   * telemetry but should NOT be used for firing decisions.
   *
   * @return true if current RPM is within tolerance of target
   */
  public boolean isAtSetpoint() {
    if (!m_isEnabled || m_targetRPM.lt(RPM.zero())) {
      return false;
    }
    return m_targetRPM.isNear(velocitySignal.getValue(), ShooterConstants.SHOOTER_SPEED_TOLERANCE);
  }

  /** @return Current flywheel velocity in RPM */
  public AngularVelocity getCurrentRPM() {
    return m_masterMotor.getVelocity().getValue();
  }

  /** @return Target flywheel velocity in RPM */
  public AngularVelocity getTargetRPM() {
    return m_targetRPM;
  }

  /** @return True if the shooter is enabled */
  public boolean isEnabled() {
    return m_isEnabled;
  }

  public double getMasterSupplyCurrentAmps() {
    return m_masterMotor.getSupplyCurrent().getValueAsDouble();
  }

  public double getSlaveSupplyCurrentAmps() {
    return m_slaveMotor.getSupplyCurrent().getValueAsDouble();
  }

  public double getMasterStatorCurrentAmps() {
    return m_masterMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getSlaveStatorCurrentAmps() {
    return m_slaveMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getMasterVoltageVolts() {
    return m_masterMotor.getMotorVoltage().getValueAsDouble();
  }

  public double getSlaveVoltageVolts() {
    return m_slaveMotor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * Command to spin up the shooter to the default RPM Ends immediately after setting the target
   * (does not wait for ready)
   */
  public Command spinUpCommand() {
    return runOnce(this::spinUp).withName("Shooter Spin Up");
  }

  /**
   * Command to spin up and wait until the shooter is ready This blocks until isReady() returns true
   */
  public Command spinUpAndWaitCommand() {
    return run(this::spinUp).until(this::isReady).withName("Shooter Spin Up & Wait");
  }

  /** Command to stop the shooter */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Shooter Stop");
  }

  /**
   * Command to hold the shooter at a specific RPM while the command runs
   *
   * @param rpm Target RPM to maintain
   */
  public Command holdRPMCommand(AngularVelocity rpm) {
    return startEnd(() -> setTargetRPM(rpm), this::stop).withName("Shooter Hold " + rpm + " RPM");
  }

  /**
   * Command to continuously track a dynamic RPM from a supplier. This is the main distance-based
   * shooting command - the RPM supplier typically comes from ShooterSetpoint via the interpolation
   * table.
   *
   * @param rpmSupplier supplier that provides the target RPM each loop
   * @return a command that continuously updates the target RPM
   */
  public Command holdRPMCommand(Supplier<AngularVelocity> rpmSupplier) {
    return run(() -> setTargetRPM(rpmSupplier.get()))
        .finallyDo(interrupted -> stop())
        .withName("Shooter Dynamic RPM");
  }

  /**
   * Check if the shooter is within a percentage tolerance of a given target RPM. Used for on-target
   * gating in the shoot command.
   *
   * @param target the target RPM to check against
   * @return true if current RPM is within ON_TARGET_RPM_PERCENT of target
   */
  public boolean isAt(AngularVelocity target) {
    if (target.lt(RPM.zero())) return false;
    return getCurrentRPM().isNear(target, ShooterConstants.ON_TARGET_RPM_PERCENT);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
