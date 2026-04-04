// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.ShooterSetpoint;
import frc.robot.lib.SmartShootController;
import frc.robot.statemachine.GameState;
import frc.robot.statemachine.RobotStateMachine;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure - the "brain" that coordinates all mechanism subsystems.
 *
 * <p>Buttons and auto routines set a {@link WantedSuperState}. Every cycle, {@link #periodic()}
 * translates that into the appropriate {@link CurrentSuperState} and drives each child subsystem's
 * wanted state accordingly.
 *
 * <p>This centralizes all mechanism coordination so DriverControls and OperatorControls only need
 * to express intent — not manage individual subsystems directly.
 */
public class Superstructure extends SubsystemBase {

  // ==================== STATE ENUMS ====================

  /** What the operator/auto wants the robot to do right now. */
  public enum WantedSuperState {
    /** Default - mechanisms off, pivot stays where it is. */
    IDLE,
    /** Deploy intake pivot + run wheels + index fuel. */
    COLLECT,
    /** Stow intake pivot + stop wheels. */
    STOW,
    /** Pre-spin flywheel + track shooter pivot angle (prepare to fire). */
    AIM,
    /** Full shoot: aim + feed indexer when flywheel, pivot, and heading are on-target. */
    SHOOT,
    /** Force-shoot: aim + feed as soon as flywheel is at speed (bypass heading/pivot gates). */
    FORCE_SHOOT,
    /** Reverse feeder/indexer without touching intake. */
    UNJAM,
    /** Extend climber for endgame. */
    CLIMB,
    /** E-stop all mechanisms. */
    STOPPED
  }

  /** What the robot is actually doing this cycle (derived from wanted + sensor feedback). */
  public enum CurrentSuperState {
    IDLE,
    COLLECTING,
    STOWING,
    AIMING,
    /** Shooter spinning up, waiting for on-target conditions. */
    WAITING_FOR_TARGET,
    /** Actively feeding - all conditions met. */
    SHOOTING,
    /** Actively feeding - force-shoot bypass. */
    FORCE_SHOOTING,
    UNJAMMING,
    CLIMBING,
    STOPPED
  }

  // ==================== SUBSYSTEM REFERENCES ====================

  private final ShooterSubsystem shooter;
  private final ShooterPivotSubsystem shooterPivot;
  private final IndexerSubsystem indexer;
  private final IntakeWheelsSubsystem intake;
  private final PivotSubsystem pivot;
  private final ClimberSubsystem climber;

  // ==================== EXTERNAL DEPENDENCIES ====================

  private final RobotStateMachine stateMachine;
  private final Supplier<ShooterSetpoint> setpointSupplier;
  private final Supplier<Boolean> headingAlignedSupplier;
  private final SmartShootController smartShootController;

  // ==================== STATE TRACKING ====================

  private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
  private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
  private CurrentSuperState previousSuperState = CurrentSuperState.IDLE;

  /**
   * When true, the Superstructure will NOT set the shooter pivot's wanted state, allowing a direct
   * command (manual control, homing) to take exclusive control.
   */
  private boolean shooterPivotOverride = false;

  // ==================== CONSTRUCTOR ====================

  public Superstructure(
      ShooterSubsystem shooter,
      ShooterPivotSubsystem shooterPivot,
      IndexerSubsystem indexer,
      IntakeWheelsSubsystem intake,
      PivotSubsystem pivot,
      ClimberSubsystem climber,
      RobotStateMachine stateMachine,
      Supplier<ShooterSetpoint> setpointSupplier,
      Supplier<Boolean> headingAlignedSupplier,
      SmartShootController smartShootController) {
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.indexer = indexer;
    this.intake = intake;
    this.pivot = pivot;
    this.climber = climber;
    this.stateMachine = stateMachine;
    this.setpointSupplier = setpointSupplier;
    this.headingAlignedSupplier = headingAlignedSupplier;
    this.smartShootController = smartShootController;
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    // Update smart shoot controller with current driver intent
    smartShootController.update(wantedSuperState == WantedSuperState.SHOOT);

    handleStateTransitions();
    applyStates();
    syncGameState();

    // Telemetry via AdvantageKit
    Logger.recordOutput("Superstructure/WantedState", wantedSuperState.name());
    Logger.recordOutput("Superstructure/CurrentState", currentSuperState.name());
    Logger.recordOutput("Superstructure/PreviousState", previousSuperState.name());
    Logger.recordOutput("Superstructure/ShooterPivotOverride", shooterPivotOverride);
    Logger.recordOutput(
        "Superstructure/SmartShoot/State", smartShootController.getState().name());
    Logger.recordOutput("Superstructure/SmartShoot/ShouldFeed", smartShootController.shouldFeed());
  }

  // ==================== PUBLIC API ====================

  public void setWantedSuperState(WantedSuperState state) {
    this.wantedSuperState = state;
  }

  public WantedSuperState getWantedSuperState() {
    return wantedSuperState;
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  /**
   * Enable/disable shooter-pivot override. When enabled, the Superstructure will skip setting the
   * shooter pivot's wanted state so a direct command (manual control, homing) can take exclusive
   * control.
   */
  public void setShooterPivotOverride(boolean override) {
    this.shooterPivotOverride = override;
  }

  // ==================== STATE TRANSITIONS ====================

  /**
   * Map the wanted super-state to the actual current super-state. Some wanted states branch based
   * on sensor feedback (e.g., SHOOT checks on-target conditions).
   */
  private void handleStateTransitions() {
    previousSuperState = currentSuperState;

    switch (wantedSuperState) {
      case IDLE -> currentSuperState = CurrentSuperState.IDLE;
      case COLLECT -> currentSuperState = CurrentSuperState.COLLECTING;
      case STOW -> currentSuperState = CurrentSuperState.STOWING;
      case AIM -> currentSuperState = CurrentSuperState.AIMING;
      case SHOOT -> {
        if (isOnTarget() && smartShootController.shouldFeed()) {
          currentSuperState = CurrentSuperState.SHOOTING;
        } else {
          currentSuperState = CurrentSuperState.WAITING_FOR_TARGET;
        }
      }
      case FORCE_SHOOT -> {
        if (isFlywheelReady() && !shooterPivot.isInTrenchZone()) {
          currentSuperState = CurrentSuperState.FORCE_SHOOTING;
        } else {
          // Still aiming - waiting for flywheel
          currentSuperState = CurrentSuperState.AIMING;
        }
      }
      case UNJAM -> currentSuperState = CurrentSuperState.UNJAMMING;
      case CLIMB -> currentSuperState = CurrentSuperState.CLIMBING;
      case STOPPED -> currentSuperState = CurrentSuperState.STOPPED;
    }
  }

  // ==================== APPLY STATES ====================

  /** Dispatch to per-state handler methods that set each child subsystem's wanted state. */
  private void applyStates() {
    // Detect state-change side effects
    if (previousSuperState == CurrentSuperState.CLIMBING
        && currentSuperState != CurrentSuperState.CLIMBING) {
      climber.setWantedState(ClimberSubsystem.WantedState.IDLE);
    }

    switch (currentSuperState) {
      case IDLE -> applyIdle();
      case COLLECTING -> applyCollect();
      case STOWING -> applyStow();
      case AIMING -> applyAim();
      case WAITING_FOR_TARGET -> applyAim(); // Same outputs - still spinning up
      case SHOOTING -> applyShoot();
      case FORCE_SHOOTING -> applyForceShoot();
      case UNJAMMING -> applyUnjam();
      case CLIMBING -> applyClimb();
      case STOPPED -> applyStopped();
    }
  }

  // ==================== STATE HANDLERS ====================

  private void applyIdle() {
    shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
    trackPivotContinuously();
    indexer.setWantedState(IndexerSubsystem.WantedState.OFF);
    intake.setWantedState(IntakeWheelsSubsystem.WantedState.OFF);
    // Pivot: don't force stow in IDLE - use STOW state explicitly
  }

  private void applyCollect() {
    pivot.setWantedState(PivotSubsystem.WantedState.DEPLOY);
    intake.setWantedState(IntakeWheelsSubsystem.WantedState.INTAKE);
    indexer.setWantedState(
        IndexerSubsystem.WantedState.OFF); // Indexer only runs during shooting/feed, never intake.
    trackPivotContinuously();
  }

  private void applyStow() {
    // STOW is pivot-only so it never interrupts active shooting/indexing behavior.
    pivot.setWantedState(PivotSubsystem.WantedState.STOW);
  }

  private void applyAim() {
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp != null && sp.isValid()) {
      shooter.setWantedState(ShooterSubsystem.WantedState.HOLD_RPM, sp.flywheelRPM());
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.TRACK_ANGLE, sp.pivotAngle());
      }
    } else {
      shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.IDLE);
      }
    }
    indexer.setWantedState(IndexerSubsystem.WantedState.OFF);
  }

  private void applyShoot() {
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp != null && sp.isValid()) {
      shooter.setWantedState(ShooterSubsystem.WantedState.HOLD_RPM, sp.flywheelRPM());
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.TRACK_ANGLE, sp.pivotAngle());
      }
    }
    // All conditions met - feed!
    indexer.setWantedState(IndexerSubsystem.WantedState.FEED);
  }

  private void applyForceShoot() {
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp != null && sp.isValid()) {
      shooter.setWantedState(ShooterSubsystem.WantedState.HOLD_RPM, sp.flywheelRPM());
      if (!shooterPivotOverride) {
        shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.TRACK_ANGLE, sp.pivotAngle());
      }
    }
    // Force feed - flywheel at speed is the only gate
    indexer.setWantedState(IndexerSubsystem.WantedState.FEED);
  }

  private void applyUnjam() {
    shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
    indexer.setWantedState(IndexerSubsystem.WantedState.REVERSE);
  }

  private void applyClimb() {
    climber.setWantedState(ClimberSubsystem.WantedState.EXTEND);
    shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
    indexer.setWantedState(IndexerSubsystem.WantedState.OFF);
    intake.setWantedState(IntakeWheelsSubsystem.WantedState.OFF);
    trackPivotContinuously();
  }

  private void applyStopped() {
    shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
    if (!shooterPivotOverride) {
      shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.IDLE);
    }
    indexer.setWantedState(IndexerSubsystem.WantedState.OFF);
    intake.setWantedState(IntakeWheelsSubsystem.WantedState.OFF);
    climber.setWantedState(ClimberSubsystem.WantedState.IDLE);
  }

  // ==================== CONTINUOUS PIVOT TRACKING ====================

  /**
   * Keep the shooter pivot tracking the distance-based angle at all times. Falls back to IDLE only
   * when there is no valid setpoint.
   */
  private void trackPivotContinuously() {
    if (shooterPivotOverride) return;
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp != null && sp.isValid()) {
      shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.TRACK_ANGLE, sp.pivotAngle());
    } else {
      shooterPivot.setWantedState(ShooterPivotSubsystem.WantedState.IDLE);
    }
  }

  // ==================== GAME STATE SYNC ====================

  /**
   * Keep the RobotStateMachine's GameState in sync with the Superstructure's current state. This
   * replaces the scattered stateMachine.setGameState() calls that were in DriverControls /
   * OperatorControls.
   */
  private void syncGameState() {
    GameState desired =
        switch (currentSuperState) {
          case COLLECTING -> GameState.COLLECTING;
          case AIMING, SHOOTING, FORCE_SHOOTING -> GameState.SCORING;
          case WAITING_FOR_TARGET -> {
            // If SmartShoot is queued (hub inactive), don't override to SCORING
            // let the RobotStateMachine's HUB_INACTIVE state stand.
            if (smartShootController.getState() == SmartShootController.SmartShootState.QUEUED) {
              yield null;
            }
            yield GameState.SCORING;
          }
          case CLIMBING -> GameState.CLIMBING;
          case UNJAMMING -> GameState.MANUAL_OVERRIDE;
          default -> null; // Don't force a game state for IDLE/STOW/STOPPED
        };

    if (desired != null && stateMachine.getGameState() != desired) {
      stateMachine.setGameState(desired);
    }
  }

  // ==================== ON-TARGET CHECKS ====================

  /** Full on-target check: flywheel RPM + pivot angle + heading alignment. */
  private boolean isOnTarget() {
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp == null || !sp.isValid()) return false;

    boolean flywheelReady = shooter.isAt(sp.flywheelRPM());
    boolean pivotReady = shooterPivot.isAtAngle(sp.pivotAngle());
    boolean headingReady = headingAlignedSupplier.get();

    Logger.recordOutput("Superstructure/OnTarget/Flywheel", flywheelReady);
    Logger.recordOutput("Superstructure/OnTarget/Pivot", pivotReady);
    Logger.recordOutput("Superstructure/OnTarget/Heading", headingReady);
    Logger.recordOutput("Superstructure/OnTarget/All", flywheelReady && pivotReady && headingReady);

    return flywheelReady && pivotReady && headingReady;
  }

  /** Flywheel-only ready check (for force-shoot gate). */
  private boolean isFlywheelReady() {
    ShooterSetpoint sp = setpointSupplier.get();
    if (sp == null || !sp.isValid()) return false;
    AngularVelocity targetRPM = sp.flywheelRPM();
    return targetRPM.gt(edu.wpi.first.units.Units.RPM.zero()) && shooter.isAt(targetRPM);
  }
}
