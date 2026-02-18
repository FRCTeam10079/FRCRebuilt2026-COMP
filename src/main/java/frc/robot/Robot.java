// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;

/**
 * Robot class for FRC 2026 REBUILT season Integrates with the Master State
 * Machine for
 * comprehensive robot control
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // MASTER STATE MACHINE - Controls EVERYTHING
  private final RobotStateMachine m_stateMachine;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_stateMachine = RobotStateMachine.getInstance();

    // ==================== LIMELIGHT CAMERA STREAMS FOR ELASTIC DASHBOARD
    // ====================
    // Add both Limelight camera streams to CameraServer so they appear in Elastic
    // dashboard
    for (String llName : VisionConstants.LIMELIGHT_NAMES) {
      try {
        HttpCamera limelightCamera = new HttpCamera(
            llName,
            "http://" + llName + ".local:5800/stream.mjpg",
            HttpCameraKind.kMJPGStreamer);
        CameraServer.startAutomaticCapture(limelightCamera);
        System.out.println(llName + " camera stream added to CameraServer");
      } catch (Exception e) {
        System.err.println("Failed to add " + llName + " camera stream: " + e.getMessage());
      }
    }
  }

  @Override
  public void robotPeriodic() {
    // Update master state machine
    m_stateMachine.periodic();

    // Run command scheduler
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // State machine transition: Robot disabled
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    // Stay in disabled state - state machine handles alliance color updates

    // ==================== LIMELIGHT 4 IMU SEEDING ====================
    // While disabled, continuously seed both Limelights' internal IMUs with robot's
    // gyro. This ensures the IMUs are synchronized before the match starts.
    double robotYaw = m_robotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble();
    for (String llName : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.SetIMUMode(llName, VisionConstants.IMU_MODE_SEED_EXTERNAL);
      LimelightHelpers.SetRobotOrientation(llName, robotYaw, 0, 0, 0, 0, 0);
    }

    // ==================== IMU SEEDING TELEMETRY ====================
    // Publish the yaw being seeded so you can verify it matches reality
    // before the match starts. Very important for MegaTag2 accuracy.
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Vision/SeedYaw", robotYaw);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(
        "Vision/IMUMode", "SEEDING (Mode 1)");
  }

  @Override
  public void disabledExit() {
    // Leaving disabled state
    System.out.println("Exiting disabled mode...");
  }

  @Override
  public void autonomousInit() {
    // State machine transition: Autonomous starting
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.AUTO_INIT);

    // ==================== LIMELIGHT 4 IMU MODE ====================
    // Switch both Limelights to internal IMU with external gyro drift correction
    // for best MegaTag2 performance
    for (String llName : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.SetIMUMode(llName, VisionConstants.IMU_MODE_INTERNAL_EXTERNAL_ASSIST);
      LimelightHelpers.SetIMUAssistAlpha(llName, VisionConstants.IMU_ASSIST_ALPHA);
    }

    // Get and schedule autonomous command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
      // Transition to running state
      m_stateMachine.setMatchState(RobotStateMachine.MatchState.AUTO_RUNNING);
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Autonomous is running - state machine tracks this
  }

  @Override
  public void autonomousExit() {
    // Autonomous ending
    System.out.println("Autonomous period ended");
  }

  @Override
  public void teleopInit() {
    // State machine transition: Teleop starting
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.TELEOP_INIT);

    // ==================== LIMELIGHT 4 IMU MODE ====================
    // Ensure both Limelights have internal IMU with external assist active for
    // teleop
    for (String llName : VisionConstants.LIMELIGHT_NAMES) {
      LimelightHelpers.SetIMUMode(llName, VisionConstants.IMU_MODE_INTERNAL_EXTERNAL_ASSIST);
      LimelightHelpers.SetIMUAssistAlpha(llName, VisionConstants.IMU_ASSIST_ALPHA);
    }

    // Cancel autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Transition to running state
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.TELEOP_RUNNING);
  }

  @Override
  public void teleopPeriodic() {
    // Teleop is running - state machine tracks endgame and hub shifts automatically
  }

  @Override
  public void teleopExit() {
    // Teleop ending
    System.out.println("Teleop period ended");
  }

  @Override
  public void testInit() {
    // State machine transition: Test mode starting
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.TEST_INIT);

    CommandScheduler.getInstance().cancelAll();

    // Transition to running state
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.TEST_RUNNING);
  }

  @Override
  public void testPeriodic() {
    // Test mode is running
  }

  @Override
  public void testExit() {
    // Test mode ending
    System.out.println("Test mode ended");
  }

  @Override
  public void simulationPeriodic() {
    // Simulation running
  }
}
