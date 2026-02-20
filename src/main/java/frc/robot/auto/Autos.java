// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.pathfinding.Pathfinding;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

/**
 * Central auto-mode configuration. Builds the SmartDashboard choosers for PathPlanner and Choreo
 * autos and provides the selected autonomous command to Robot.java.
 */
public class Autos {

  private final CommandSwerveDrivetrain drivetrain;
  private final AutoFactory choreoAutoFactory;

  private final SendableChooser<Command> pathPlannerChooser;
  private final SendableChooser<String> choreoChooser = new SendableChooser<>();

  /**
   * Create the auto configuration and publish choosers to SmartDashboard.
   *
   * @param drivetrain the swerve drivetrain
   * @param choreoAutoFactory the Choreo AutoFactory (already created in RobotContainer)
   */
  public Autos(CommandSwerveDrivetrain drivetrain, AutoFactory choreoAutoFactory) {
    this.drivetrain = drivetrain;
    this.choreoAutoFactory = choreoAutoFactory;

    // PathPlanner chooser — populates from deploy/pathplanner/autos/
    pathPlannerChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", pathPlannerChooser);

    // Choreo chooser — manually populated with known trajectories
    configureChoreoChooser();
    SmartDashboard.putData("Choreo Auto", choreoChooser);
  }

  // ==================== CHOOSER BUILDERS ====================

  private void configureChoreoChooser() {
    choreoChooser.setDefaultOption("RS", "RS");
    choreoChooser.addOption("LS_Depot", "LS_Depot");
    choreoChooser.addOption("LS_Neutral", "LS_Neutral");
    choreoChooser.addOption("MS_Depot_Climb", "MS_Depot_Climb");
    choreoChooser.addOption("None", "");
    choreoChooser.addOption("Right_OutPost", "Right_OutPost");
  }

  // ==================== AUTO COMMAND SELECTION ====================

  /**
   * Returns the autonomous command selected by the dashboard. Priority:
   *
   * <ol>
   *   <li>Choreo trajectory (if a non-blank name is selected)
   *   <li>PathPlanner auto (from the auto chooser)
   *   <li>Fallback: AD* pathfind to AprilTag 10
   * </ol>
   */
  public Command getSelected() {
    // 1) Choreo
    String choreoName = choreoChooser.getSelected();
    if (choreoName != null && !choreoName.isBlank()) {
      return getChoreoAuto(choreoName);
    }

    // 2) PathPlanner
    Command ppAuto = pathPlannerChooser.getSelected();
    if (ppAuto != null) {
      return ppAuto;
    }

    // 3) Fallback
    return getRecoveryPath();
  }

  /**
   * Build a Choreo autonomous routine for the given trajectory name.
   *
   * @param trajectoryName the Choreo trajectory file name (e.g. "RS_Outpost")
   * @return a command that resets odometry and follows the trajectory
   */
  public Command getChoreoAuto(String trajectoryName) {
    AutoRoutine routine = choreoAutoFactory.newRoutine("SelectedChoreo");
    AutoTrajectory trajectory = routine.trajectory(trajectoryName);
    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine.cmd().withName("Choreo: " + trajectoryName);
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
