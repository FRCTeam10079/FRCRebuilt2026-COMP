// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoCommands;
import frc.robot.auto.Autos;
import frc.robot.controllers.DriverControls;
import frc.robot.controllers.OperatorControls;
import frc.robot.controllers.TestingBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.pathfinding.Pathfinding;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * RobotContainer for FRC 2026 REBUILT season This class is where the robot's subsystems, commands,
 * and button bindings are defined.
 */
public class RobotContainer {

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final CommandXboxController m_testController = new CommandXboxController(2);

  // State Machine
  private final RobotStateMachine m_stateMachine = RobotStateMachine.getInstance();

  // ==================== SUBSYSTEMS ====================
  // Drivetrain - created from TunerConstants
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Vision
  public final VisionSubsystem vision;
  // Indexer
  private final IndexerSubsystem indexer = new IndexerSubsystem();

  // Mechanisms

  public final ShooterSubsystem shooter = new ShooterSubsystem();
  // Intake
  private final IntakeWheelsSubsystem intake = new IntakeWheelsSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();

  // Climber (stub — hardware not wired yet)
  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final Telemetry m_telemetry =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  // ==================== AUTO ====================
  private final AutoFactory choreoAutoFactory;
  private final AutoCommands autoCommands;
  private final Autos autos;

  public RobotContainer() {
    // Create vision subsystem (needs drivetrain reference for pose injection)
    vision = new VisionSubsystem(drivetrain);

    drivetrain.registerTelemetry(m_telemetry::telemeterize);

    // Register controllers with state machine for haptic feedback
    m_stateMachine.registerControllers(m_driverController, m_operatorController);

    // Initialize the pathfinding system
    initializePathfinding();

    choreoAutoFactory = new AutoFactory(
        () -> drivetrain.getState().Pose,
        drivetrain::resetPose,
        drivetrain::followTrajectory,
        false,
        drivetrain);

    // ==================== REGISTER NAMED COMMANDS ====================
    // AutoCommands provides DRY factory methods used by both PathPlanner and
    // Choreo.
    // Must be registered BEFORE any PathPlanner autos/paths are created.
    autoCommands = new AutoCommands(intake, pivot, indexer, shooter, drivetrain, vision);
    autoCommands.registerPathPlannerCommands();
    autoCommands.registerChoreoBindings(choreoAutoFactory);

    // ==================== BUILD AUTO CHOOSER ====================
    autos = new Autos(drivetrain, choreoAutoFactory);

    // Configure button bindings
    configureBindings();
  }

  /**
   * Initialize the pathfinding system. This loads the navgrid and starts the background AD*
   * planning thread.
   */
  private void initializePathfinding() {
    DataLogManager.log("[RobotContainer] Initializing pathfinding system...");
    Pathfinding.ensureInitialized();
    DataLogManager.log("[RobotContainer] Pathfinding system ready");
  }

  /**
   * Configure button bindings for driver and operator controllers. Delegates to dedicated binding
   * classes for clean separation.
   */
  private void configureBindings() {
    DriverControls.configure(
        m_driverController, drivetrain, vision, intake, pivot, shooter, indexer, m_stateMachine);
    OperatorControls.configure(
        m_operatorController, intake, pivot, indexer, climber, m_stateMachine);
    TestingBindings.configure(
        m_testController, drivetrain, intake, pivot, indexer, shooter, vision);
  }

  /** Get the driver controller for use in commands/subsystems */
  public CommandXboxController getDriverController() {
    return m_driverController;
  }

  /** Get the operator controller for use in commands/subsystems */
  public CommandXboxController getOperatorController() {
    return m_operatorController;
  }

  /** Get the state machine instance */
  public RobotStateMachine getStateMachine() {
    return m_stateMachine;
  }

  /**
   * Returns the autonomous command selected via SmartDashboard.
   *
   * @see Autos#getSelected()
   */
  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
}
