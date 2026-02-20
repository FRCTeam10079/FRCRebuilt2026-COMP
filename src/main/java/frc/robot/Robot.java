// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.statemachine.MatchState;
import frc.robot.statemachine.RobotStateMachine;

/**
 * Robot class for FRC 2026 REBUILT season Integrates with the Master State Machine for
 * comprehensive robot control
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // MASTER STATE MACHINE - Controls EVERYTHING
  private final RobotStateMachine m_stateMachine;

  public Robot() {
    // Start structured data logging — logs are written to /home/lvuser/logs on the
    // roboRIO.
    DataLogManager.start();

    m_robotContainer = new RobotContainer();
    m_stateMachine = RobotStateMachine.getInstance();

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
    m_stateMachine.setMatchState(MatchState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    // Send heading to Limelights (Mode 0 = EXTERNAL_ONLY, no LL4 internal IMU).
    // Also checks MT1 multi-tag heading for auto-correction of pose estimator.
    m_robotContainer.vision.updateWhileDisabled();
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

    // Vision uses Mode 0 (EXTERNAL_ONLY) — no IMU mode switch needed.
    // Heading is sent every frame in VisionSubsystem.periodic().

    // Get and schedule autonomous command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
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

    // Vision uses Mode 0 (EXTERNAL_ONLY) — no IMU mode switch needed.
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
