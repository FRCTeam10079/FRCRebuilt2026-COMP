// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterPivotSubsystem extends SubsystemBase {

  private final ShooterPivotIO io;
  private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  // State tracking
  private boolean m_isHomed = true;
  private Angle m_targetAngleDegrees = ShooterPivotConstants.MIN_ANGLE;

  // Trench auto-lower
  private final Supplier<Pose2d> m_poseSupplier;
  private boolean m_trenchMode = false;

  public ShooterPivotSubsystem(ShooterPivotIO io, Supplier<Pose2d> poseSupplier) {
    this.io = io;
    m_poseSupplier = poseSupplier;
  }

  // ==================== TRENCH ZONE DETECTION ====================

  public boolean isInTrenchZone() {
    if (m_poseSupplier == null) return false;
    Pose2d pose = m_poseSupplier.get();
    if (pose == null) return false;

    Distance x = Meters.of(pose.getX());
    Distance y = Meters.of(pose.getY());
    Distance fieldW = ShooterPivotConstants.FIELD_WIDTH_METERS;
    Distance margin = ShooterPivotConstants.TRENCH_APPROACH_MARGIN;

    if (m_trenchMode) {
      boolean inX =
          x.gte(ShooterPivotConstants.TRENCH_X_MIN) && x.lte(ShooterPivotConstants.TRENCH_X_MAX);
      boolean inY = y.lte(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD)
          || y.gte(fieldW.minus(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD));
      return inX && inY;
    } else {
      boolean inX = x.gte(ShooterPivotConstants.TRENCH_X_MIN.minus(margin))
          && x.lte(ShooterPivotConstants.TRENCH_X_MAX.plus(margin));
      boolean inY = y.lte(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD.plus(margin))
          || y.gte(fieldW.minus(ShooterPivotConstants.TRENCH_Y_WALL_THRESHOLD).minus(margin));
      return inX && inY;
    }
  }

  // ==================== POSITION CONTROL ====================

  public void setAngle(Angle angle) {
    if (m_trenchMode) {
      m_trenchMode = isInTrenchZone();
    } else if (isInTrenchZone()) {
      lowerForTrenchZone();
    } else {
      setAngleUnchecked(
          Constants.clamp(angle, ShooterPivotConstants.MIN_ANGLE, ShooterPivotConstants.MAX_ANGLE));
    }
  }

  private void setAngleUnchecked(Angle angle) {
    m_targetAngleDegrees = angle;
    io.setMotionMagicPosition(ShooterPivotConstants.degreesToMotorRotations(
            m_targetAngleDegrees.minus(ShooterPivotConstants.MIN_ANGLE))
        .in(Rotations));
  }

  private void lowerForTrenchZone() {
    m_trenchMode = true;
    if (m_targetAngleDegrees.gt(ShooterPivotConstants.TRENCH_LOWER_ANGLE)) {
      setAngleUnchecked(ShooterPivotConstants.TRENCH_LOWER_ANGLE);
    }
  }

  public Angle getCurrentAngle() {
    return ShooterPivotConstants.motorRotationsToDegrees(Rotations.of(inputs.positionRotations))
        .plus(ShooterPivotConstants.MIN_ANGLE);
  }

  public boolean isAtAngle(Angle targetDegrees) {
    return getCurrentAngle().isNear(targetDegrees, ShooterPivotConstants.SHOOTING_TOLERANCE);
  }

  public boolean isAtTarget() {
    return isAtAngle(m_targetAngleDegrees);
  }

  public boolean isHomed() {
    return m_isHomed;
  }

  public Angle getTargetAngleDegrees() {
    return m_targetAngleDegrees;
  }

  // ==================== MANUAL / RAW CONTROL ====================

  public void setOutput(double output) {
    double clamped = MathUtil.clamp(
        output, -ShooterPivotConstants.MANUAL_MAX_OUTPUT, ShooterPivotConstants.MANUAL_MAX_OUTPUT);
    io.setDutyCycle(clamped);
  }

  public void stop() {
    io.setNeutral();
  }

  public double getPosition() {
    return inputs.positionRotations;
  }

  public double getVelocity() {
    return inputs.velocityRPS;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getStatorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  public double getMotorVoltageVolts() {
    return inputs.voltageVolts;
  }

  public void reZeroIfNeeded() {
    if (inputs.positionRotations < 0.0) {
      io.setEncoderPosition(0);
    }
  }

  // ==================== HOMING ====================

  private void enableSoftwareLimits() {
    var softLimits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ShooterPivotConstants.degreesToMotorRotations(
            ShooterPivotConstants.MAX_ANGLE.minus(ShooterPivotConstants.MIN_ANGLE)));
    io.applySoftwareLimits(softLimits);
  }

  public Command homeCommand() {
    final int[] stallCounter = {0};

    return Commands.sequence(
            Commands.runOnce(() -> {
              m_isHomed = false;
              stallCounter[0] = 0;
            }),
            run(() -> {
                  io.setDutyCycle(ShooterPivotConstants.HOMING_SPEED);

                  if (Amps.of(inputs.statorCurrentAmps)
                      .gt(ShooterPivotConstants.HOMING_CURRENT_THRESHOLD)) {
                    stallCounter[0]++;
                  } else {
                    stallCounter[0] = 0;
                  }
                })
                .until(() -> stallCounter[0] >= ShooterPivotConstants.HOMING_STALL_CYCLES),
            Commands.runOnce(() -> {
              io.setEncoderPosition(0);
              m_isHomed = true;
              enableSoftwareLimits();
              m_targetAngleDegrees = ShooterPivotConstants.MIN_ANGLE;
            }),
            Commands.runOnce(this::stop))
        .withName("ShooterPivot Home");
  }

  // ==================== COMMANDS ====================

  public Command trackAngleCommand(Supplier<Angle> angleSupplier) {
    return run(() -> setAngle(angleSupplier.get()))
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot Track Angle");
  }

  public Command goToAngleCommand(Angle angleDegrees) {
    return run(() -> setAngle(angleDegrees))
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot GoTo " + angleDegrees + "deg");
  }

  public Command manualControlCommand(DoubleSupplier axisSupplier) {
    return run(() -> {
          double raw = axisSupplier.getAsDouble();
          double deadbanded = MathUtil.applyDeadband(raw, ShooterPivotConstants.MANUAL_DEADBAND);
          setOutput(deadbanded * ShooterPivotConstants.MANUAL_MAX_OUTPUT);
        })
        .finallyDo(interrupted -> stop())
        .withName("ShooterPivot Manual");
  }

  public Command zeroEncoderCommand() {
    return runOnce(() -> io.setEncoderPosition(0)).withName("ShooterPivot Zero Encoder");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterPivot", inputs);

    // Trench safety: actively lower pivot when entering trench zone
    if (m_trenchMode) {
      m_trenchMode = isInTrenchZone();
    } else if (isInTrenchZone()) {
      lowerForTrenchZone();
    }

    Logger.recordOutput("ShooterPivot/isInTrenchZone", isInTrenchZone());
    Logger.recordOutput("ShooterPivot/AngleDegrees", getCurrentAngle().in(Degrees));
    Logger.recordOutput("ShooterPivot/TargetAngleDegrees", m_targetAngleDegrees.in(Degrees));
    Logger.recordOutput("ShooterPivot/Position", getPosition());
    Logger.recordOutput("ShooterPivot/Velocity", getVelocity());
    Logger.recordOutput("ShooterPivot/IsHomed", m_isHomed);
    Logger.recordOutput("ShooterPivot/AtTarget", isAtTarget());
    Logger.recordOutput("ShooterPivot/TrenchMode", m_trenchMode);
  }
}
