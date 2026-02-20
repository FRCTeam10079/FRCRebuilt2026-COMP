// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.AlignPosition;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.EndIntakingCommand;
import frc.robot.commands.IntakeFuelCommand;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Factory for autonomous command compositions. Each method returns a <b>new</b> Command instance so
 * it can be safely registered with both PathPlanner NamedCommands and Choreo AutoFactory bindings
 * (WPILib commands cannot be shared across multiple triggers).
 *
 * <p>This class eliminates the massive duplication that previously existed between
 * registerNamedCommands() and registerChoreoBindings() in RobotContainer.
 */
public class AutoCommands {

  private final IntakeWheelsSubsystem intake;
  private final PivotSubsystem pivot;
  private final IndexerSubsystem indexer;
  private final ShooterSubsystem shooter;
  private final CommandSwerveDrivetrain drivetrain;
  private final VisionSubsystem vision;

  public AutoCommands(
      IntakeWheelsSubsystem intake,
      PivotSubsystem pivot,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      CommandSwerveDrivetrain drivetrain,
      VisionSubsystem vision) {
    this.intake = intake;
    this.pivot = pivot;
    this.indexer = indexer;
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.vision = vision;
  }

  // ==================== INTAKE WHEELS ====================

  /** Run intake motors to collect game pieces (runs until cancelled). */
  public Command intake() {
    return intake.intakeInCommand();
  }

  /** Reverse intake wheels (unjam / eject). */
  public Command intakeOut() {
    return intake.intakeOutCommand();
  }

  /** Stop intake wheels immediately. */
  public Command stopIntake() {
    return intake.stopCommand();
  }

  // ==================== PIVOT ====================

  /** Deploy the intake pivot arm to pickup position, waits until setpoint reached. */
  public Command deployPivot() {
    return pivot.deployCommand();
  }

  /** Stow the intake pivot arm, waits until setpoint reached. */
  public Command stowPivot() {
    return pivot.stowCommand();
  }

  /** Deploy pivot + spin intake. Ends when pivot is deployed. */
  public Command intakeFuel() {
    return new IntakeFuelCommand(pivot, intake);
  }

  /** Stow pivot + stop intake. Ends when pivot is stowed. */
  public Command endIntaking() {
    return new EndIntakingCommand(pivot, intake);
  }

  // ==================== INDEXER ====================

  /** Feed game pieces forward through the indexer. */
  public Command runIndexer() {
    return indexer.feedCommand();
  }

  /** Reverse indexer to unjam. */
  public Command reverseIndexer() {
    return indexer.reverseCommand();
  }

  /** Stop indexer immediately. */
  public Command stopIndexer() {
    return indexer.stopCommand();
  }

  // ==================== SHOOTER ====================

  /** Spin up flywheel to target RPM and hold. */
  public Command spinUpShooter() {
    return shooter.holdRPMCommand(Constants.ShooterConstants.SHOOTER_SPINUP_RPM);
  }

  /** Stop shooter immediately. */
  public Command stopShooter() {
    return shooter.stopCommand();
  }

  /** Spin up shooter, wait until ready, then feed with indexer. */
  public Command shoot() {
    return shooter
        .holdRPMCommand(Constants.ShooterConstants.SHOOTER_SPINUP_RPM)
        .alongWith(Commands.waitUntil(shooter::isReady)
            .andThen(
                indexer.feedCommand().withTimeout(Constants.ShooterConstants.SHOOT_FEED_TIMEOUT)));
  }

  // ==================== VISION ALIGNMENT ====================

  /** Align to AprilTag — CENTER position. */
  public Command alignCenter() {
    return new AlignToAprilTag(drivetrain, vision, AlignPosition.CENTER);
  }

  /** Align to AprilTag — LEFT position. */
  public Command alignLeft() {
    return new AlignToAprilTag(drivetrain, vision, AlignPosition.LEFT);
  }

  /** Align to AprilTag — RIGHT position. */
  public Command alignRight() {
    return new AlignToAprilTag(drivetrain, vision, AlignPosition.RIGHT);
  }

  // ==================== STOP ALL ====================

  /** Stop all mechanisms at once. */
  public Command stopAll() {
    return Commands.runOnce(
        () -> {
          intake.stop();
          indexer.stop();
          shooter.stop();
        },
        intake,
        indexer,
        shooter);
  }

  // ==================== BULK REGISTRATION ====================

  /**
   * Register all named commands for PathPlanner autonomous routines. Must be called BEFORE any
   * PathPlannerAuto or AutoBuilder.buildAutoChooser() calls.
   */
  public void registerPathPlannerCommands() {
    // Intake
    NamedCommands.registerCommand("intake", intake());
    NamedCommands.registerCommand("intakeOut", intakeOut());
    NamedCommands.registerCommand("stopIntake", stopIntake());

    // Pivot
    NamedCommands.registerCommand("deployPivot", deployPivot());
    NamedCommands.registerCommand("stowPivot", stowPivot());

    // Combined intake + pivot
    NamedCommands.registerCommand("intakeFuel", intakeFuel());
    NamedCommands.registerCommand("endIntaking", endIntaking());

    // Indexer
    NamedCommands.registerCommand("runIndexer", runIndexer());
    NamedCommands.registerCommand("reverseIndexer", reverseIndexer());
    NamedCommands.registerCommand("stopIndexer", stopIndexer());

    // Shooter
    NamedCommands.registerCommand("spinUpShooter", spinUpShooter());
    NamedCommands.registerCommand("stopShooter", stopShooter());
    NamedCommands.registerCommand("shoot", shoot());

    // Vision alignment
    NamedCommands.registerCommand("alignCenter", alignCenter());
    NamedCommands.registerCommand("alignLeft", alignLeft());
    NamedCommands.registerCommand("alignRight", alignRight());

    // Stop all
    NamedCommands.registerCommand("stopAll", stopAll());

    // Aliases matching capitalized names used in PathPlanner auto files
    // (Right_OutPost.auto uses "Intake down", "Shoot", "Intake")
    NamedCommands.registerCommand("Intake down", intakeFuel());
    NamedCommands.registerCommand("Shoot", shoot());
    NamedCommands.registerCommand("Intake", intake());

    DataLogManager.log("[AutoCommands] Named commands registered for PathPlanner");
  }

  /**
   * Register Choreo global marker bindings for subsystem actions. These bindings are evaluated from
   * event markers inside Choreo trajectories.
   */
  public void registerChoreoBindings(AutoFactory factory) {
    factory
        // Intake
        .bind("intake", intake())
        .bind("intakeOut", intakeOut())
        .bind("stopIntake", stopIntake())
        // Pivot
        .bind("deployPivot", deployPivot())
        .bind("stowPivot", stowPivot())
        // Combined intake + pivot
        .bind("intakeFuel", intakeFuel())
        .bind("endIntaking", endIntaking())
        // Indexer
        .bind("runIndexer", runIndexer())
        .bind("reverseIndexer", reverseIndexer())
        .bind("stopIndexer", stopIndexer())
        // Shooter
        .bind("spinUpShooter", spinUpShooter())
        .bind("stopShooter", stopShooter())
        .bind("shoot", shoot())
        // Vision alignment
        .bind("alignCenter", alignCenter())
        .bind("alignLeft", alignLeft())
        .bind("alignRight", alignRight())
        // Stop all
        .bind("stopAll", stopAll())
        // Aliases matching auto file names
        .bind("Intake down", intakeFuel())
        .bind("Shoot", shoot())
        .bind("Intake", intake());

    DataLogManager.log("[AutoCommands] Marker bindings registered for Choreo");
  }
}
