// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.pathfinding.Pathfinding;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

/**
 * Central auto-mode configuration. Builds the SmartDashboard choosers for PathPlanner and Choreo
 * autos and provides the selected autonomous command to Robot.java.
 *
 * <p>Uses AutoFactory.trajectoryCmd() / resetOdometry() for simple command composition instead of
 * AutoRoutine.
 */
public class Autos {

  private final CommandSwerveDrivetrain drivetrain;
  private final AutoCommands autoCommands;

  private final SendableChooser<Command> pathPlannerChooser;

  /**
   * Create the auto configuration and publish choosers to SmartDashboard.
   *
   * @param drivetrain the swerve drivetrain
   * @param choreoAutoFactory the Choreo AutoFactory (already created in RobotContainer)
   */
  public Autos(CommandSwerveDrivetrain drivetrain, AutoCommands autoCommands) {
    this.drivetrain = drivetrain;
    this.autoCommands = autoCommands;

    // PathPlanner chooser — populates from deploy/pathplanner/autos/
    pathPlannerChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", pathPlannerChooser);
  }

  // ==================== AUTO COMMAND SELECTION ====================

  /**
   * Returns the autonomous command selected by the dashboard. Priority:
   *
   * <ol>
   *   <li>PathPlanner auto (from the auto chooser)
   *   <li>Fallback: AD* pathfind to AprilTag 10
   * </ol>
   */
  public Command getSelected() {
    Command ppAuto = pathPlannerChooser.getSelected();
    if (ppAuto == null) {
      DataLogManager.log("NO AUTO SELECTED, MISSING PATHPLANNER CONFIG");
      return Commands.none();
    }
    return ppAuto;
  }

  /**
   * Emergency recovery path — AD* pathfind to AprilTag 10. Use when no auto is selected or as a
   * fallback.
   *
   * @return a pathfinding command targeting AprilTag 10
   */
  public Command getRecoveryPath() {
    Pathfinding.setAutoObstacles();
    return drivetrain.pathfindToAprilTag10().withName("Fallback: Pathfind to Tag 10");
  }
}
