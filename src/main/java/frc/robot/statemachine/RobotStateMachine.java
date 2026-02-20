// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachine;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;

/**
 * MASTER ROBOT STATE MACHINE - FRC 2026 REBUILT
 *
 * <p>Tracks the robot's lifecycle phase (MatchState), high-level strategy (GameState), drivetrain
 * control mode, hub-shift timing, climb progress, and fuel inventory.
 *
 * <p>Robot.java drives MatchState transitions; game-level states are set by commands/subsystems as
 * mechanisms come online. Telemetry is published to SmartDashboard every cycle.
 */
public class RobotStateMachine extends SubsystemBase {

  // ===== SINGLETON =====
  private static RobotStateMachine instance;

  public static RobotStateMachine getInstance() {
    if (instance == null) {
      instance = new RobotStateMachine();
    }
    return instance;
  }

  // ===== CURRENT STATE =====
  private MatchState matchState = MatchState.DISABLED;
  private DrivetrainMode driveMode = DrivetrainMode.FIELD_CENTRIC;
  private GameState gameState = GameState.IDLE;
  private FuelState fuelState = FuelState.EMPTY;
  private HubShiftState hubShiftState = HubShiftState.UNKNOWN;
  private ClimbState climbState = ClimbState.NOT_CLIMBING;

  // ===== TRACKING =====
  private boolean isAlignedToTarget = false;
  private boolean isShooterAtRPM = false;
  private double stateStartTime = 0;
  private double matchStartTime = 0;
  private MatchState previousMatchState = MatchState.DISABLED;
  private GameState previousGameState = GameState.IDLE;
  private int fuelCount = 0;

  // ===== CYCLE STATISTICS =====
  private int fuelScoredAuto = 0;
  private int fuelScoredTeleop = 0;
  private int intakeCyclesCompleted = 0;
  private int scoringCyclesCompleted = 0;
  private double lastCycleTime = 0;
  private double fastestCycleTime = Double.MAX_VALUE;

  // ===== STATE HISTORY =====
  private static final int STATE_HISTORY_SIZE = Constants.StateMachineConstants.STATE_HISTORY_SIZE;
  private final List<StateHistoryEntry> stateHistory = new ArrayList<>();

  // ===== DRIVER FEEDBACK =====
  private CommandXboxController driverController;
  private CommandXboxController operatorController;
  private double rumbleEndTime = 0;
  private boolean hasFiredCriticalTimeWarning = false;

  // Telemetry throttle — publish at ~5 Hz (every 10th cycle at 50 Hz)
  private static final int TELEMETRY_DIVISOR = 10;
  private int telemetryCycleCount = 0;

  /** State history entry for debugging. */
  private record StateHistoryEntry(double timestamp, String type, String from, String to) {
    @Override
    public String toString() {
      return String.format("[%.2f] %s: %s -> %s", timestamp, type, from, to);
    }
  }

  // ==================== CONSTRUCTOR ====================

  private RobotStateMachine() {}

  // ==================== STATE TRANSITIONS ====================

  /** Transition match state (called by Robot.java). */
  public void setMatchState(MatchState newState) {
    if (matchState != newState) {
      previousMatchState = matchState;
      matchState = newState;
      stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      onMatchStateChange();
      logStateChange("Match", previousMatchState.name(), newState.name());
    }
  }

  /** Transition game state. */
  public void setGameState(GameState newState) {
    if (gameState != newState) {
      previousGameState = gameState;
      gameState = newState;
      stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      onGameStateChange();
      logStateChange("Game", previousGameState.name(), newState.name());
    }
  }

  /** Safe game state transition with validation. */
  public void requestGameState(GameState newState) {
    if (newState == GameState.EMERGENCY_STOP || canTransitionTo(newState)) {
      setGameState(newState);
    } else {
      System.err.println("INVALID STATE TRANSITION: " + gameState + " -> " + newState);
    }
  }

  /** Transition drivetrain mode. */
  public void setDrivetrainMode(DrivetrainMode newMode) {
    if (driveMode != newMode) {
      DrivetrainMode prev = driveMode;
      driveMode = newMode;
      logStateChange("Drivetrain", prev.name(), newMode.name());
    }
  }

  /** Set hub shift state. */
  public void setHubShiftState(HubShiftState newState) {
    if (hubShiftState != newState) {
      HubShiftState prev = hubShiftState;
      hubShiftState = newState;
      logStateChange("HubShift", prev.name(), newState.name());
      onHubShiftChange();
    }
  }

  /** Set climb state. */
  public void setClimbState(ClimbState newState) {
    if (climbState != newState) {
      ClimbState prev = climbState;
      climbState = newState;
      logStateChange("Climb", prev.name(), newState.name());

      if (newState == ClimbState.ENGAGED) {
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_MAX,
            Constants.StateMachineConstants.RUMBLE_EXTRA_LONG);
        DataLogManager.log("=== CLIMB COMPLETE - BRAKE ENGAGED ===");
      } else if (newState == ClimbState.FAILED) {
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_MEDIUM_INTENSITY,
            Constants.StateMachineConstants.RUMBLE_LONG);
        DataLogManager.log("!!! CLIMB FAILED - RECOVERY NEEDED !!!");
      }
    }
  }

  // ==================== INTERNAL TRANSITION LOGIC ====================

  private void onMatchStateChange() {
    switch (matchState) {
      case DISABLED -> {
        setGameState(GameState.IDLE);
        setDrivetrainMode(DrivetrainMode.DISABLED);
        isAlignedToTarget = false;
        isShooterAtRPM = false;
        climbState = ClimbState.NOT_CLIMBING;
      }
      case AUTO_INIT -> {
        setGameState(GameState.AUTO_RUNNING);
        setDrivetrainMode(DrivetrainMode.PATH_FOLLOWING);
        isAlignedToTarget = false;
        resetCycleCounters();
      }
      case TELEOP_INIT -> {
        setGameState(GameState.IDLE);
        setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);
        isAlignedToTarget = false;
        hubShiftState = HubShiftState.UNKNOWN;
        hasFiredCriticalTimeWarning = false;
      }
      case TRANSITION_SHIFT -> {
        setHubShiftState(HubShiftState.TRANSITION);
        setGameState(GameState.TRANSITION);
        DataLogManager.log("=== TRANSITION SHIFT - BOTH HUBS ACTIVE! ===");
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_MAX,
            Constants.StateMachineConstants.RUMBLE_LONG);
      }
      case ENDGAME -> {
        DataLogManager.log("=== ENDGAME PERIOD STARTED! ===");
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_STRONG,
            Constants.StateMachineConstants.RUMBLE_LONG);
      }
      default -> {}
    }
  }

  private void onGameStateChange() {
    switch (gameState) {
      case EMERGENCY_STOP -> setDrivetrainMode(DrivetrainMode.DISABLED);
      case CLIMBING, CLIMBED -> setDrivetrainMode(DrivetrainMode.LOCKED);
      case DEFENDING -> setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);
      case SCORING -> setDrivetrainMode(DrivetrainMode.VISION_TRACKING);
      default -> {} // Other states do not force a drivetrain mode
    }
  }

  private void onHubShiftChange() {
    switch (hubShiftState) {
      case MY_HUB_ACTIVE -> {
        if (!gameState.isEndgame() && !gameState.isClimbing()) {
          setGameState(GameState.HUB_ACTIVE);
          rumbleDriver(
              Constants.StateMachineConstants.RUMBLE_MEDIUM_INTENSITY,
              Constants.StateMachineConstants.RUMBLE_MEDIUM);
        }
      }
      case MY_HUB_INACTIVE -> {
        if (!gameState.isEndgame() && !gameState.isClimbing()) {
          setGameState(GameState.HUB_INACTIVE);
          rumbleDriver(
              Constants.StateMachineConstants.RUMBLE_LIGHT,
              Constants.StateMachineConstants.RUMBLE_SHORT);
        }
      }
      case TRANSITION ->
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_MAX,
            Constants.StateMachineConstants.RUMBLE_LONG);
      default -> {}
    }
  }

  /** Validate whether a game state transition is legal. */
  private boolean canTransitionTo(GameState newState) {
    if (matchState == MatchState.DISABLED && newState != GameState.IDLE) {
      return false;
    }
    if (newState.isAuto() && !matchState.autonomous) {
      return false;
    }
    return true;
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    checkPeriodTransitions();
    updateRumble();

    // Throttle telemetry to ~5 Hz (every TELEMETRY_DIVISOR cycles)
    if (++telemetryCycleCount >= TELEMETRY_DIVISOR) {
      telemetryCycleCount = 0;
      updateTelemetry();
    }
  }

  /** Detect endgame / transition periods automatically from match time. */
  private void checkPeriodTransitions() {
    if (matchState == MatchState.TELEOP_RUNNING) {
      if (isEndgamePeriod()) {
        setMatchState(MatchState.ENDGAME);
      } else if (isTransitionPeriod()) {
        setMatchState(MatchState.TRANSITION_SHIFT);
      }
    }

    // === Critical time warning (10 seconds remaining) ===
    if (isTeleop() && !hasFiredCriticalTimeWarning) {
      double t = DriverStation.getMatchTime();
      if (t > 0 && t <= Constants.GameConstants.CRITICAL_TIME_THRESHOLD) {
        hasFiredCriticalTimeWarning = true;
        DataLogManager.log("!!! CRITICAL: 10 SECONDS REMAINING !!!");
        rumbleBoth(
            Constants.StateMachineConstants.RUMBLE_MAX,
            Constants.StateMachineConstants.RUMBLE_EXTRA_LONG);
      }
    }
  }

  // ==================== TELEMETRY ====================

  private void updateTelemetry() {
    // Match lifecycle
    SmartDashboard.putString("Match State", matchState.name());
    SmartDashboard.putBoolean("Robot Enabled", matchState.enabled);
    SmartDashboard.putBoolean("Is Autonomous", matchState.autonomous);

    // Game strategy
    SmartDashboard.putString("Game State", gameState.name());
    SmartDashboard.putString("Game Description", gameState.description);

    // Drivetrain
    SmartDashboard.putString("Drivetrain Mode", driveMode.description);

    // Hub shift
    SmartDashboard.putString("Hub Shift State", hubShiftState.name());
    SmartDashboard.putBoolean("My Hub Active", hubShiftState == HubShiftState.MY_HUB_ACTIVE);

    // Climb
    SmartDashboard.putString("Climb State", climbState.name());
    SmartDashboard.putBoolean("Climb Complete", climbState.isCompleted());

    // Fuel
    SmartDashboard.putNumber("Fuel Count", fuelCount);
    SmartDashboard.putBoolean("Has Fuel", hasFuel());

    // Shooter / Alignment
    SmartDashboard.putBoolean("Aligned to Target", isAlignedToTarget);
    SmartDashboard.putBoolean("Shooter at RPM", isShooterAtRPM);
    SmartDashboard.putBoolean("Ready to Fire", isReadyToFire());

    // Alliance & Timing
    SmartDashboard.putString(
        "Alliance", DriverStation.getAlliance().map(Enum::name).orElse("UNKNOWN"));
    SmartDashboard.putNumber("Time in State", getTimeInState());

    // Cycle stats
    SmartDashboard.putNumber("Total Fuel Scored", getTotalFuelScored());
    SmartDashboard.putNumber("Fastest Cycle Time", getFastestCycleTime());
  }

  // ==================== GETTERS ====================

  public MatchState getMatchState() {
    return matchState;
  }

  public GameState getGameState() {
    return gameState;
  }

  public DrivetrainMode getDrivetrainMode() {
    return driveMode;
  }

  /**
   * Get the current alliance from DriverStation.
   *
   * @return the current alliance, or empty if unknown
   */
  public java.util.Optional<Alliance> getAlliance() {
    return DriverStation.getAlliance();
  }

  public FuelState getFuelState() {
    return fuelState;
  }

  public HubShiftState getHubShiftState() {
    return hubShiftState;
  }

  public ClimbState getClimbState() {
    return climbState;
  }

  public int getFuelCount() {
    return fuelCount;
  }

  public boolean isEnabled() {
    return matchState.enabled;
  }

  public boolean isAutonomous() {
    return matchState.autonomous;
  }

  public boolean isTeleop() {
    return matchState == MatchState.TELEOP_RUNNING
        || matchState == MatchState.TRANSITION_SHIFT
        || matchState == MatchState.ENDGAME;
  }

  public double getTimeInState() {
    return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - stateStartTime;
  }

  public boolean isEndgamePeriod() {
    return DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() > 0
        && DriverStation.getMatchTime() <= Constants.GameConstants.ENDGAME_THRESHOLD;
  }

  public boolean isTransitionPeriod() {
    if (!DriverStation.isTeleopEnabled()) return false;
    double t = DriverStation.getMatchTime();
    return t >= Constants.GameConstants.TRANSITION_END_TIME
        && t <= Constants.GameConstants.TRANSITION_START_TIME;
  }

  // ==================== ALIGNMENT / SHOOTER TRACKING ====================

  public void setAlignedToTarget(boolean aligned) {
    if (isAlignedToTarget != aligned) {
      isAlignedToTarget = aligned;
      if (aligned) {
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_LIGHT,
            Constants.StateMachineConstants.RUMBLE_SHORT);
      }
    }
  }

  public boolean isAlignedToTarget() {
    return isAlignedToTarget;
  }

  public void setShooterAtRPM(boolean atRPM) {
    isShooterAtRPM = atRPM;
  }

  public boolean isShooterAtRPM() {
    return isShooterAtRPM;
  }

  public boolean isReadyToFire() {
    return hasFuel()
        && isAlignedToTarget
        && isShooterAtRPM
        && (hubShiftState == HubShiftState.MY_HUB_ACTIVE
            || hubShiftState == HubShiftState.TRANSITION);
  }

  // ==================== FUEL MANAGEMENT ====================

  public boolean hasFuel() {
    return fuelCount > 0;
  }

  public boolean isEmpty() {
    return fuelCount == 0;
  }

  public void setFuelState(FuelState newState) {
    if (fuelState != newState) {
      FuelState prev = fuelState;
      fuelState = newState;
      addToStateHistory("Fuel", prev.name(), newState.name());

      if (newState == FuelState.LOADED) {
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_LIGHT,
            Constants.StateMachineConstants.RUMBLE_SHORT);
        intakeCyclesCompleted++;
      } else if (newState == FuelState.EMPTY && prev == FuelState.FIRING) {
        rumbleDriver(
            Constants.StateMachineConstants.RUMBLE_MAX,
            Constants.StateMachineConstants.RUMBLE_LONG);
        if (isAutonomous()) {
          fuelScoredAuto += fuelCount;
        } else {
          fuelScoredTeleop += fuelCount;
        }
        scoringCyclesCompleted++;
        updateCycleTime();
      }
    }
  }

  public void setFuelCount(int count) {
    fuelCount = Math.max(0, count);
    if (fuelCount == 0) {
      setFuelState(FuelState.EMPTY);
    } else {
      setFuelState(FuelState.LOADED);
    }
  }

  public void addFuel(int amount) {
    setFuelCount(fuelCount + amount);
  }

  public void removeFuel(int amount) {
    setFuelCount(fuelCount - amount);
  }

  // ==================== DRIVER FEEDBACK ====================

  public void registerControllers(CommandXboxController driver, CommandXboxController operator) {
    this.driverController = driver;
    this.operatorController = operator;
  }

  public void rumbleDriver(double intensity, double durationSeconds) {
    if (driverController != null) {
      driverController.getHID().setRumble(RumbleType.kBothRumble, intensity);
      rumbleEndTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() + durationSeconds;
    }
  }

  public void rumbleOperator(double intensity, double durationSeconds) {
    if (operatorController != null) {
      operatorController.getHID().setRumble(RumbleType.kBothRumble, intensity);
      rumbleEndTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() + durationSeconds;
    }
  }

  /** Rumble both controllers simultaneously. */
  public void rumbleBoth(double intensity, double durationSeconds) {
    rumbleDriver(intensity, durationSeconds);
    rumbleOperator(intensity, durationSeconds);
  }

  private void updateRumble() {
    if (rumbleEndTime > 0 && edu.wpi.first.wpilibj.Timer.getFPGATimestamp() >= rumbleEndTime) {
      if (driverController != null) {
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
      if (operatorController != null) {
        operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
      rumbleEndTime = 0;
    }
  }

  // ==================== STATE HISTORY ====================

  private void logStateChange(String type, String from, String to) {
    String message = String.format("%s State: %s -> %s", type, from, to);
    DataLogManager.log(message);
    SmartDashboard.putString("Last State Change", message);
    addToStateHistory(type, from, to);
  }

  private void addToStateHistory(String type, String from, String to) {
    stateHistory.add(
        new StateHistoryEntry(edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), type, from, to));
    if (stateHistory.size() > STATE_HISTORY_SIZE) {
      stateHistory.remove(0);
    }
  }

  public List<StateHistoryEntry> getStateHistory() {
    return new ArrayList<>(stateHistory);
  }

  public void printStateHistory() {
    DataLogManager.log("=== STATE HISTORY ===");
    stateHistory.forEach(entry -> DataLogManager.log(entry.toString()));
    DataLogManager.log("====================");
  }

  // ==================== CYCLE STATISTICS ====================

  private void updateCycleTime() {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (lastCycleTime > 0) {
      double cycleTime = now - lastCycleTime;
      if (cycleTime < fastestCycleTime
          && cycleTime > Constants.StateMachineConstants.MIN_VALID_CYCLE_TIME) {
        fastestCycleTime = cycleTime;
      }
    }
    lastCycleTime = now;
  }

  public int getFuelScoredAuto() {
    return fuelScoredAuto;
  }

  public int getFuelScoredTeleop() {
    return fuelScoredTeleop;
  }

  public int getTotalFuelScored() {
    return fuelScoredAuto + fuelScoredTeleop;
  }

  public int getIntakeCyclesCompleted() {
    return intakeCyclesCompleted;
  }

  public int getScoringCyclesCompleted() {
    return scoringCyclesCompleted;
  }

  public double getFastestCycleTime() {
    return fastestCycleTime == Double.MAX_VALUE ? 0 : fastestCycleTime;
  }

  public double getMatchElapsedTime() {
    if (matchStartTime == 0) return 0;
    return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - matchStartTime;
  }

  public void resetCycleCounters() {
    fuelScoredAuto = 0;
    fuelScoredTeleop = 0;
    intakeCyclesCompleted = 0;
    scoringCyclesCompleted = 0;
    lastCycleTime = 0;
    fastestCycleTime = Double.MAX_VALUE;
    fuelCount = 0;
    stateHistory.clear();
    matchStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }
}
