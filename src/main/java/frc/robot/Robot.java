// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.LaunchCalculator;
import frc.robot.lib.PowerDiagnosticsLogger;
import frc.robot.lib.ShooterMath;
import frc.robot.statemachine.MatchState;
import frc.robot.statemachine.RobotStateMachine;

/**
 * Robot class for FRC 2026 REBUILT season Integrates with the Master State Machine for
 * comprehensive robot control
 */
public class Robot extends TimedRobot {
  private static final String VOLTAGE_KEY = "Voltage";
  private static final String MATCH_TIME_KEY = "Match Time";
  private static final String AUTO_DELAY_KEY = "Auto Delay";
  private static final String ROBOT_VELOCITY_KEY = "Robot Velocity";

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // MASTER STATE MACHINE - Controls EVERYTHING
  private final RobotStateMachine m_stateMachine;
  private final PowerDiagnosticsLogger m_powerDiagnosticsLogger;

  public Robot() {
    // Start structured data logging - logs are written to /home/lvuser/logs on the
    // roboRIO.
    DataLogManager.start();

    m_robotContainer = new RobotContainer();
    m_stateMachine = RobotStateMachine.getInstance();
    m_powerDiagnosticsLogger = new PowerDiagnosticsLogger(
        m_robotContainer.getIntake(),
        m_robotContainer.getPivot(),
        m_robotContainer.getIndexer(),
        m_robotContainer.shooter,
        m_robotContainer.shooterPivot);

    // ==================== LIMELIGHT CAMERA STREAMS FOR ELASTIC DASHBOARD
    // ====================
    var nt = NetworkTableInstance.getDefault();
    for (String llName : Constants.VisionConstants.LIMELIGHT_NAMES) {
      StringArrayPublisher pub = nt.getTable("/CameraPublisher/" + llName)
          .getStringArrayTopic("streams")
          .publish();
      pub.set(new String[] {"mjpg:http://" + llName + ".local:5800/stream.mjpg"});
      DataLogManager.log(llName + " stream URL published to NetworkTables");
    }

    SmartDashboard.setDefaultNumber(AUTO_DELAY_KEY, 0.0);
  }

  @Override
  public void robotPeriodic() {
    // Clear shoot-on-the-move parameters at the start of each cycle.
    // They will be re-populated by the shootOnTheMoveDriveCommand if active.
    LaunchCalculator.getInstance().clearParameters();

    SmartDashboard.putNumber(
        "Shooter/Distance To Hub (Meters)",
        ShooterMath.getDistanceToHub(m_robotContainer.drivetrain.getState().Pose)
            .in(Meters));

    SmartDashboard.putNumber(VOLTAGE_KEY, RobotController.getBatteryVoltage());

    // WPILib returns -1 when the DS is enabled without FMS. On real FMS this counts
    // down in
    // whole-second steps.
    double matchTimeSeconds = DriverStation.getMatchTime();
    SmartDashboard.putNumber(MATCH_TIME_KEY, matchTimeSeconds < 0.0 ? 0.0 : matchTimeSeconds);

    ChassisSpeeds chassisSpeeds = m_robotContainer.drivetrain.getState().Speeds;
    double robotVelocityMetersPerSecond =
        Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(ROBOT_VELOCITY_KEY, robotVelocityMetersPerSecond);

    // Update master state machine
    m_stateMachine.periodic();

    // Run command scheduler
    CommandScheduler.getInstance().run();

    m_powerDiagnosticsLogger.logPeriodic();
  }

  @Override
  public void disabledInit() {
    // State machine transition: Robot disabled
    m_stateMachine.setMatchState(MatchState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.vision.updateWhileDisabled();
    m_robotContainer.shooterPivot.reZeroIfNeeded();
  }

  @Override
  public void disabledExit() {
    // Leaving disabled state
    DataLogManager.log("Exiting disabled mode...");
  }

  @Override
  public void autonomousInit() {
    // State machine transition: Autonomous starting
    m_stateMachine.setMatchState(MatchState.AUTO_INIT);

    // Vision uses Mode 0 (EXTERNAL_ONLY) - no IMU mode switch needed.
    // Heading is sent every frame in VisionSubsystem.periodic().

    // Get and schedule autonomous command
    Command selectedAuto = m_robotContainer.getAutonomousCommand();
    double autoDelaySeconds = SmartDashboard.getNumber(AUTO_DELAY_KEY, 0.0);

    if (selectedAuto != null) {
      m_autonomousCommand = autoDelaySeconds > 0.0
          ? Commands.waitSeconds(autoDelaySeconds).andThen(selectedAuto)
          : selectedAuto;
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
      // Transition to running state
      m_stateMachine.setMatchState(MatchState.AUTO_RUNNING);
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Autonomous is running - state machine tracks this
  }

  @Override
  public void autonomousExit() {
    // Autonomous ending
    DataLogManager.log("Autonomous period ended");
  }

  @Override
  public void teleopInit() {
    // State machine transition: Teleop starting
    m_stateMachine.setMatchState(MatchState.TELEOP_INIT);

    // Vision uses Mode 0 (EXTERNAL_ONLY) - no IMU mode switch needed.
    // Heading is sent every frame in VisionSubsystem.periodic().

    // Cancel autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Transition to running state
    m_stateMachine.setMatchState(MatchState.TELEOP_RUNNING);
  }

  @Override
  public void teleopPeriodic() {
    // Teleop is running - state machine tracks endgame and hub shifts automatically
  }

  @Override
  public void teleopExit() {
    // Teleop ending
    DataLogManager.log("Teleop period ended");
  }

  @Override
  public void testInit() {
    // State machine transition: Test mode starting
    m_stateMachine.setMatchState(MatchState.TEST_INIT);

    CommandScheduler.getInstance().cancelAll();

    // Transition to running state
    m_stateMachine.setMatchState(MatchState.TEST_RUNNING);
  }

  @Override
  public void testPeriodic() {
    // Test mode is running
  }

  @Override
  public void testExit() {
    // Test mode ending
    DataLogManager.log("Test mode ended");
  }

  @Override
  public void simulationPeriodic() {
    // Simulation running
  }
}
