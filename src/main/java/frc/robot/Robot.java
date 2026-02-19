// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
      System.out.println(llName + " stream URL published to NetworkTables");
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
    // Continuously seed the LL4 IMU with the robot's current heading.
    // Mode 1 (Seed) overwrites the LL4 internal IMU heading so it is
    // correct by the time the match starts.
    m_robotContainer.vision.seedIMU();
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

    // Switch LL4 IMU to external-assist mode for 1kHz heading during match
    m_robotContainer.vision.enableIMU();

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

    // Switch LL4 IMU to external-assist mode for 1kHz heading during match
    m_robotContainer.vision.enableIMU();

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
