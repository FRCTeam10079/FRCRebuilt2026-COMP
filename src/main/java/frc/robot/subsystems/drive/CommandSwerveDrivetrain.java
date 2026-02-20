// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.drive.SwerveHeadingController.HeadingControllerState;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  // Skew compensation and deadband sourced from Constants
  private static final double SKEW_COMPENSATION_SCALAR =
      Constants.DrivetrainConstants.SKEW_COMPENSATION_SCALAR;
  private static final double CONTROLLER_DEADBAND = Constants.DrivetrainConstants.DEADBAND_PERCENT;

  /*
   * Velocity coefficients for dynamic speed control These allow runtime
   * adjustment of speeds (e.g., slow mode, scoring mode) Range: 0.0 (stopped) to
   * 1.0 (full speed)
   */
  private double teleopVelocityCoefficient = 1.0;
  private double rotationVelocityCoefficient = 1.0;

  // ==================== HEADING CONTROLLER ====================
  private final SwerveHeadingController m_headingController = new SwerveHeadingController();
  private boolean m_headingLockEnabled = false;
  private double m_headingLockTarget = 0.0;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  // ==================== CHOREO TRAJECTORY FOLLOWER ====================
  private final PIDController m_choreoXController = new PIDController(
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KP,
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KI,
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KD);
  private final PIDController m_choreoYController = new PIDController(
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KP,
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KI,
      Constants.DrivetrainConstants.CHOREO_TRANSLATION_KD);
  private final PIDController m_choreoHeadingController = new PIDController(
      Constants.DrivetrainConstants.CHOREO_HEADING_KP,
      Constants.DrivetrainConstants.CHOREO_HEADING_KI,
      Constants.DrivetrainConstants.CHOREO_HEADING_KD);

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default
          // ramp rate
          // (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /**
   * Resets the robot's field-relative heading to the alliance-correct forward direction while
   * preserving the current XY position from the pose estimator.
   *
   * <p>This is the correct "reset heading" for drivers: it tells the pose estimator "I am facing
   * the field" (0 deg for Blue alliance, 180 deg for Red alliance) without destroying vision
   * localization. MegaTag2 immediately adapts because it reads the fused heading from
   * getState().Pose each frame via SetRobotOrientation().
   *
   * <p>Inspired by Team 6328 (Mechanical Advantage) approach: adjust gyro offset to map old
   * rotation to new rotation, preserving XY.
   */
  public void resetFieldHeading() {
    Pose2d currentPose = getState().Pose;

    // Determine the correct "forward" heading based on alliance
    // Blue alliance: 0 deg means facing the red alliance wall
    // Red alliance: 180 deg means facing the blue alliance wall
    Rotation2d allianceForwardHeading =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Rotation2d.k180deg
            : Rotation2d.kZero;

    // Build a new pose that keeps the current XY but sets the heading
    // to the alliance-correct forward direction
    Pose2d correctedPose = new Pose2d(currentPose.getTranslation(), allianceForwardHeading);

    // resetPose() recalibrates the Pigeon2 fused heading offset so the
    // pose estimator's heading matches our desired heading. XY is preserved.
    resetPose(correctedPose);

    DataLogManager.log("[Heading] Reset field heading: "
        + currentPose.getRotation().getDegrees() + "° -> "
        + allianceForwardHeading.getDegrees() + "° (XY preserved: "
        + String.format("%.2f, %.2f", currentPose.getX(), currentPose.getY()) + ")");
    Logger.recordOutput("Vision/HeadingResetOldDeg", currentPose.getRotation().getDegrees());
    Logger.recordOutput("Vision/HeadingResetNewDeg", allianceForwardHeading.getDegrees());
  }

  /*
   * SysId routine for characterizing rotation. This is used to find PID gains for
   * the FieldCentricFacingAngle HeadingController. See the documentation of
   * SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> {
            /* output is actually radians per second, but SysId only supports "volts" */
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) -> setControl(m_pathApplyRobotSpeeds
              .withSpeeds(speeds)
              .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
              .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(
                  Constants.DrivetrainConstants.PP_TRANSLATION_KP,
                  Constants.DrivetrainConstants.PP_TRANSLATION_KI,
                  Constants.DrivetrainConstants.PP_TRANSLATION_KD),
              // PID constants for rotation
              new PIDConstants(
                  Constants.DrivetrainConstants.PP_ROTATION_KP,
                  Constants.DrivetrainConstants.PP_ROTATION_KI,
                  Constants.DrivetrainConstants.PP_ROTATION_KD)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the
          // case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for
          // requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Follow a Choreo swerve sample. This method is passed into Choreo's AutoFactory.
   *
   * @param sample Choreo sample at the current autonomous timestamp
   */
  public void followTrajectory(SwerveSample sample) {
    Pose2d currentPose = getState().Pose;

    ChassisSpeeds targetFieldSpeeds = new ChassisSpeeds(
        sample.vx + m_choreoXController.calculate(currentPose.getX(), sample.x),
        sample.vy + m_choreoYController.calculate(currentPose.getY(), sample.y),
        sample.omega
            + m_choreoHeadingController.calculate(
                currentPose.getRotation().getRadians(), sample.heading));

    setControl(m_fieldCentricRequest.withSpeeds(targetFieldSpeeds));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by
   * {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by
   * {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective. If we haven't applied the
     * operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match. Otherwise, only check and apply the operator perspective if the DS
     * is disabled. This ensures driving behavior doesn't change until an explicit
     * disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(allianceColor -> {
        setOperatorPerspectiveForward(
            allianceColor == Alliance.Red
                ? kRedAlliancePerspectiveRotation
                : kBlueAlliancePerspectiveRotation);
        m_hasAppliedOperatorPerspective = true;
      });
    }

    Logger.recordOutput("Drive Pos", getState().Pose);
  }

  {
    m_choreoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ==================== VISION MEASUREMENT OVERRIDES ====================
  // These overrides convert FPGA timestamps to current time for the Kalman filter
  // Following the official CTRE Phoenix6 2026 examples pattern

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to
   * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  /**
   * Return the pose at a given timestamp, if the buffer is not empty.
   *
   * @param timestampSeconds The timestamp of the pose in seconds.
   * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
   */
  @Override
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  // ==================== SMOOTH DRIVING METHODS ====================

  /**
   * Creates a SwerveRequest for smooth teleop field-centric driving.
   *
   * <p>Uses OpenLoopVoltage for more responsive driving feel during teleop (as opposed to Velocity
   * mode which can feel sluggish).
   */
  private final SwerveRequest.ApplyFieldSpeeds m_fieldCentricRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  /**
   * Calculate chassis speeds with skew compensation for smooth driving.
   *
   * <p>Skew compensation corrects for the drift that occurs when both translating and rotating
   * simultaneously. The robot's actual heading is predicted slightly ahead based on current
   * rotational velocity.
   *
   * @param xVelocity Field-relative X velocity (m/s, positive = toward opposing alliance)
   * @param yVelocity Field-relative Y velocity (m/s, positive = left)
   * @param angularVelocity Angular velocity (rad/s, positive = counter-clockwise)
   * @return ChassisSpeeds with skew compensation applied
   */
  public ChassisSpeeds calculateSpeedsWithSkewCompensation(
      double xVelocity, double yVelocity, double angularVelocity) {

    var currentPose = getState().Pose;
    var currentSpeeds = getState().Speeds;

    // Calculate skew compensation factor based on current angular velocity
    Rotation2d skewCompensationFactor =
        Rotation2d.fromRadians(currentSpeeds.omegaRadiansPerSecond * SKEW_COMPENSATION_SCALAR);

    // Convert field-relative speeds to robot-relative, then back to field-relative
    // with the skew compensation applied
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xVelocity, yVelocity, angularVelocity), currentPose.getRotation()),
        currentPose.getRotation().plus(skewCompensationFactor));
  }

  /**
   * Apply field-centric driving with smooth driving tuning.
   *
   * <p>Features: - Deadband application to eliminate joystick drift - Squared angular input for
   * finer low-speed rotation control - Skew compensation for smooth combined translation/rotation -
   * OpenLoopVoltage mode for responsive feel - Runtime velocity coefficients for slow mode /
   * scoring mode
   *
   * @param xInput Raw X joystick input (-1 to 1)
   * @param yInput Raw Y joystick input (-1 to 1)
   * @param rotationInput Raw rotation joystick input (-1 to 1)
   * @param maxVelocity Maximum translation velocity (m/s)
   * @param maxAngularVelocity Maximum angular velocity (rad/s)
   */
  public void driveFieldCentricSmooth(
      double xInput,
      double yInput,
      double rotationInput,
      double maxVelocity,
      double maxAngularVelocity) {

    // Apply deadband to eliminate joystick drift
    double xMagnitude = MathUtil.applyDeadband(xInput, CONTROLLER_DEADBAND);
    double yMagnitude = MathUtil.applyDeadband(yInput, CONTROLLER_DEADBAND);
    double angularMagnitude = MathUtil.applyDeadband(rotationInput, CONTROLLER_DEADBAND);

    // Square the angular magnitude for finer low-speed control
    // while maintaining direction (sign)
    angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

    // Calculate velocities (flip for alliance if needed)
    // Apply velocity coefficients for runtime speed adjustment (slow mode, etc.)
    boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    double xVelocity =
        (isBlueAlliance ? -xMagnitude : xMagnitude) * maxVelocity * teleopVelocityCoefficient;
    double yVelocity =
        (isBlueAlliance ? -yMagnitude : yMagnitude) * maxVelocity * teleopVelocityCoefficient;
    double angularVelocity = -angularMagnitude * maxAngularVelocity * rotationVelocityCoefficient;

    // Apply skew compensation for smooth combined translation/rotation
    ChassisSpeeds compensatedSpeeds =
        calculateSpeedsWithSkewCompensation(xVelocity, yVelocity, angularVelocity);

    // Apply to drivetrain using OpenLoopVoltage for responsive feel
    setControl(m_fieldCentricRequest.withSpeeds(compensatedSpeeds));
  }

  /**
   * Set the translation velocity coefficient for teleop driving. Use this for slow mode, scoring
   * mode, etc.
   *
   * @param coefficient Speed multiplier (0.0 = stopped, 1.0 = full speed)
   */
  public void setTeleopVelocityCoefficient(double coefficient) {
    this.teleopVelocityCoefficient = MathUtil.clamp(coefficient, 0.0, 1.0);
  }

  /**
   * Set the rotation velocity coefficient for teleop driving.
   *
   * @param coefficient Rotation speed multiplier (0.0 = no rotation, 1.0 = full rotation)
   */
  public void setRotationVelocityCoefficient(double coefficient) {
    this.rotationVelocityCoefficient = MathUtil.clamp(coefficient, 0.0, 1.0);
  }

  /** Get the current translation velocity coefficient. */
  public double getTeleopVelocityCoefficient() {
    return teleopVelocityCoefficient;
  }

  /**
   * Creates a command for smooth teleop driving
   *
   * @param xInputSupplier Supplier for X joystick input (typically leftY, inverted)
   * @param yInputSupplier Supplier for Y joystick input (typically leftX, inverted)
   * @param rotationInputSupplier Supplier for rotation joystick input (typically rightX)
   * @param maxVelocity Maximum translation velocity (m/s)
   * @param maxAngularVelocity Maximum angular velocity (rad/s)
   * @return Command that continuously applies smooth driving
   */
  public Command smoothTeleopDriveCommand(
      Supplier<Double> xInputSupplier,
      Supplier<Double> yInputSupplier,
      Supplier<Double> rotationInputSupplier,
      double maxVelocity,
      double maxAngularVelocity) {

    return run(() -> driveFieldCentricSmooth(
        xInputSupplier.get(),
        yInputSupplier.get(),
        rotationInputSupplier.get(),
        maxVelocity,
        maxAngularVelocity));
  }

  // ==================== HEADING LOCK CONTROL ====================

  /** Get the heading controller for external configuration */
  public SwerveHeadingController getHeadingController() {
    return m_headingController;
  }

  /**
   * Enable heading lock mode with a specific target heading. In this mode, the robot will
   * automatically rotate to face the target heading while still allowing the driver to control
   * translation.
   *
   * @param targetDegrees The target heading in degrees (field-relative)
   */
  public void enableHeadingLock(double targetDegrees) {
    m_headingLockEnabled = true;
    m_headingLockTarget = targetDegrees;
    m_headingController.setGoal(targetDegrees);
    m_headingController.setHeadingControllerState(HeadingControllerState.SNAP);
  }

  /** Update the target heading without resetting the controller state */
  public void updateHeadingLockTarget(double targetDegrees) {
    m_headingLockTarget = targetDegrees;
    m_headingController.setGoal(targetDegrees);
  }

  /** Disable heading lock mode */
  public void disableHeadingLock() {
    m_headingLockEnabled = false;
    m_headingController.setHeadingControllerState(HeadingControllerState.OFF);
  }

  /**
   * Check if the robot is currently locked to the target heading within tolerance
   *
   * @return true if heading lock is enabled and robot is at target
   */
  public boolean isHeadingLocked() {
    return m_headingLockEnabled && m_headingController.isAtGoal();
  }

  /**
   * Get the current heading lock target
   *
   * @return Target heading in degrees
   */
  public double getHeadingLockTarget() {
    return m_headingLockTarget;
  }

  /**
   * Apply field-centric driving WITH heading lock.
   *
   * <p>The driver controls translation (X/Y), but rotation is automatically controlled by the
   * heading controller to maintain the locked heading. This creates a "turret mode" where the robot
   * faces a specific direction regardless of how the driver is strafing.
   *
   * @param xInput Raw X joystick input (-1 to 1)
   * @param yInput Raw Y joystick input (-1 to 1)
   * @param maxVelocity Maximum translation velocity (m/s)
   * @param maxAngularVelocity Maximum angular velocity (rad/s)
   */
  public void driveWithHeadingLock(
      double xInput, double yInput, double maxVelocity, double maxAngularVelocity) {

    // Apply deadband to translation inputs
    double xMagnitude = MathUtil.applyDeadband(xInput, CONTROLLER_DEADBAND);
    double yMagnitude = MathUtil.applyDeadband(yInput, CONTROLLER_DEADBAND);

    // Calculate translation velocities (flip for alliance)
    boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    double xVelocity =
        (isBlueAlliance ? -xMagnitude : xMagnitude) * maxVelocity * teleopVelocityCoefficient;
    double yVelocity =
        (isBlueAlliance ? -yMagnitude : yMagnitude) * maxVelocity * teleopVelocityCoefficient;

    // Get current heading
    double currentHeadingDegrees = getState().Pose.getRotation().getDegrees();

    // Calculate rotation output from heading controller (-1 to 1)
    double rotationOutput = m_headingController.update(currentHeadingDegrees);

    // Convert to angular velocity
    double angularVelocity = rotationOutput * maxAngularVelocity;

    // Apply skew compensation
    ChassisSpeeds compensatedSpeeds =
        calculateSpeedsWithSkewCompensation(xVelocity, yVelocity, angularVelocity);

    // Apply to drivetrain
    setControl(m_fieldCentricRequest.withSpeeds(compensatedSpeeds));

    // Log heading controller telemetry
    m_headingController.logTelemetry(currentHeadingDegrees);
  }

  /**
   * Creates a command for heading-locked driving.
   *
   * <p>The driver controls translation with the left stick, but the robot automatically rotates to
   * face the specified target heading.
   *
   * @param xInputSupplier Supplier for X joystick input
   * @param yInputSupplier Supplier for Y joystick input
   * @param targetHeadingSupplier Supplier for target heading in degrees
   * @param maxVelocity Maximum translation velocity (m/s)
   * @param maxAngularVelocity Maximum angular velocity (rad/s)
   * @return Command that applies heading-locked driving
   */
  public Command headingLockedDriveCommand(
      DoubleSupplier xInputSupplier,
      DoubleSupplier yInputSupplier,
      DoubleSupplier targetHeadingSupplier,
      double maxVelocity,
      double maxAngularVelocity) {

    return run(() -> {
          // Update target heading each loop (allows dynamic targeting like AprilTag
          // tracking)
          double targetHeading = targetHeadingSupplier.getAsDouble();
          if (!m_headingLockEnabled) {
            enableHeadingLock(targetHeading);
          } else {
            updateHeadingLockTarget(targetHeading);
          }

          driveWithHeadingLock(
              xInputSupplier.getAsDouble(),
              yInputSupplier.getAsDouble(),
              maxVelocity,
              maxAngularVelocity);
        })
        .finallyDo(this::disableHeadingLock);
  }

  /**
   * Creates a command for heading-locked driving to a FIXED heading.
   *
   * @param xInputSupplier Supplier for X joystick input
   * @param yInputSupplier Supplier for Y joystick input
   * @param fixedTargetHeading Fixed target heading in degrees
   * @param maxVelocity Maximum translation velocity (m/s)
   * @param maxAngularVelocity Maximum angular velocity (rad/s)
   * @return Command that applies heading-locked driving to the fixed heading
   */
  public Command headingLockedDriveCommand(
      DoubleSupplier xInputSupplier,
      DoubleSupplier yInputSupplier,
      double fixedTargetHeading,
      double maxVelocity,
      double maxAngularVelocity) {

    return headingLockedDriveCommand(
        xInputSupplier, yInputSupplier, () -> fixedTargetHeading, maxVelocity, maxAngularVelocity);
  }

  // ==================== PATHFINDING COMMANDS ====================

  /**
   * Create a command to pathfind to AprilTag 10 (Red Alliance Hub Face).
   *
   * <p>Uses the AD* pathfinding algorithm to find a safe path around obstacles, then follows the
   * path using PID control.
   *
   * @return Command that pathfinds and drives to the scoring position in front of AprilTag 10
   */
  public Command pathfindToAprilTag10() {
    return frc.robot.pathfinding.PathfindToTagCommand.toAprilTag10(this);
  }

  /**
   * Create a command to pathfind to a specific AprilTag.
   *
   * @param tagId The AprilTag ID to target
   * @return Command that pathfinds and drives to the scoring position
   */
  public Command pathfindToAprilTag(int tagId) {
    return frc.robot.pathfinding.PathfindToTagCommand.toAprilTag(this, tagId);
  }

  /**
   * Create a command to pathfind to a specific pose.
   *
   * @param targetPose The target pose to pathfind to
   * @return Command that pathfinds and drives to the target
   */
  public Command pathfindToPose(edu.wpi.first.math.geometry.Pose2d targetPose) {
    return new frc.robot.pathfinding.PathfindToTagCommand(
        this, () -> targetPose, frc.robot.pathfinding.PathConstraints.DEFAULT);
  }

  /**
   * Create a command to pathfind to a dynamically-supplied pose.
   *
   * @param targetPoseSupplier Supplier for the target pose
   * @return Command that pathfinds and drives to the target
   */
  public Command pathfindToPose(Supplier<edu.wpi.first.math.geometry.Pose2d> targetPoseSupplier) {
    return new frc.robot.pathfinding.PathfindToTagCommand(
        this, targetPoseSupplier, frc.robot.pathfinding.PathConstraints.DEFAULT);
  }
}
