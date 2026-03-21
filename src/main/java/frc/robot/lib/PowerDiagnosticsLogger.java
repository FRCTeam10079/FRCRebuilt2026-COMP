package frc.robot.lib;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PowerDiagnosticsLogger {
  private static final double LOG_PERIOD_SECONDS = 0.1;
  private static final double FAILURE_LOG_PERIOD_SECONDS = 2.0;
  private static final double PDH_RETRY_BASE_SECONDS = 0.5;
  private static final double PDH_RETRY_MAX_SECONDS = 5.0;
  private static final double SUMMARY_LOG_PERIOD_SECONDS = 1.0;
  private static final double BATTERY_LOW_THRESHOLD_VOLTS = 9.0;
  private static final double HIGH_MECHANISM_CURRENT_THRESHOLD_AMPS = 120.0;
  private static final double CURRENT_SPIKE_THRESHOLD_AMPS = 40.0;

  private final PowerDistribution m_powerDistribution;
  private final IntakeWheelsSubsystem m_intake;
  private final PivotSubsystem m_intakePivot;
  private final IndexerSubsystem m_indexer;
  private final ShooterSubsystem m_shooter;
  private final ShooterPivotSubsystem m_shooterPivot;

  private final DoubleLogEntry m_batteryVoltageEntry;
  private final DoubleLogEntry m_brownoutVoltageEntry;
  private final DoubleLogEntry m_pdhVoltageEntry;
  private final DoubleLogEntry m_pdhTemperatureEntry;
  private final DoubleLogEntry m_pdhTotalCurrentEntry;
  private final DoubleLogEntry m_pdhTotalPowerEntry;
  private final DoubleLogEntry m_pdhTotalEnergyEntry;
  private final BooleanLogEntry m_pdhSwitchableChannelEntry;

  private final DoubleLogEntry[] m_channelCurrentEntries;

  private final DoubleLogEntry m_intakeSupplyCurrentEntry;
  private final DoubleLogEntry m_intakeStatorCurrentEntry;
  private final DoubleLogEntry m_intakeVoltageEntry;

  private final DoubleLogEntry m_intakePivotSupplyCurrentEntry;
  private final DoubleLogEntry m_intakePivotStatorCurrentEntry;
  private final DoubleLogEntry m_intakePivotVoltageEntry;

  private final DoubleLogEntry m_indexerFeederSupplyCurrentEntry;
  private final DoubleLogEntry m_indexerSpindexerSupplyCurrentEntry;
  private final DoubleLogEntry m_indexerFeederStatorCurrentEntry;
  private final DoubleLogEntry m_indexerSpindexerStatorCurrentEntry;
  private final DoubleLogEntry m_indexerFeederVoltageEntry;
  private final DoubleLogEntry m_indexerSpindexerVoltageEntry;
  private final DoubleLogEntry m_indexerTotalSupplyCurrentEntry;

  private final DoubleLogEntry m_shooterMasterSupplyCurrentEntry;
  private final DoubleLogEntry m_shooterSlaveSupplyCurrentEntry;
  private final DoubleLogEntry m_shooterMasterStatorCurrentEntry;
  private final DoubleLogEntry m_shooterSlaveStatorCurrentEntry;
  private final DoubleLogEntry m_shooterMasterVoltageEntry;
  private final DoubleLogEntry m_shooterSlaveVoltageEntry;
  private final DoubleLogEntry m_shooterTotalSupplyCurrentEntry;

  private final DoubleLogEntry m_shooterPivotSupplyCurrentEntry;
  private final DoubleLogEntry m_shooterPivotStatorCurrentEntry;
  private final DoubleLogEntry m_shooterPivotVoltageEntry;

  private final DoubleLogEntry m_mechanismsTotalSupplyCurrentEntry;
  private final StringLogEntry m_diagnosticSummaryEntry;
  private final StringLogEntry m_diagnosticEventEntry;

  private double m_lastLogTimestampSeconds = Double.NEGATIVE_INFINITY;
  private double m_lastFailureLogTimestampSeconds = Double.NEGATIVE_INFINITY;
  private double m_lastSummaryTimestampSeconds = Double.NEGATIVE_INFINITY;
  private boolean m_pdhTelemetryEnabled = true;
  private int m_pdhFailureCount = 0;
  private double m_nextPdhRetryTimestampSeconds = 0.0;
  private double m_lastPdhTotalCurrentAmps = Double.NaN;
  private double m_lastPdhVoltageVolts = Double.NaN;
  private double m_lastMechanismTotalAmps = Double.NaN;
  private boolean m_batteryLowActive = false;
  private boolean m_highCurrentActive = false;

  public PowerDiagnosticsLogger(
      IntakeWheelsSubsystem intake,
      PivotSubsystem intakePivot,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      ShooterPivotSubsystem shooterPivot) {
    m_intake = intake;
    m_intakePivot = intakePivot;
    m_indexer = indexer;
    m_shooter = shooter;
    m_shooterPivot = shooterPivot;

    PowerDistribution pdh = null;
    try {
      pdh = new PowerDistribution();
    } catch (RuntimeException ex) {
      DataLogManager.log("[PowerDiagnostics] PD init failed: " + ex.getMessage());
    }
    m_powerDistribution = pdh;

    DataLog log = DataLogManager.getLog();

    m_batteryVoltageEntry = new DoubleLogEntry(log, "/power/robot/batteryVoltageVolts");
    m_brownoutVoltageEntry = new DoubleLogEntry(log, "/power/robot/brownoutVoltageVolts");
    m_pdhVoltageEntry = new DoubleLogEntry(log, "/power/pdh/voltageVolts");
    m_pdhTemperatureEntry = new DoubleLogEntry(log, "/power/pdh/temperatureCelsius");
    m_pdhTotalCurrentEntry = new DoubleLogEntry(log, "/power/pdh/totalCurrentAmps");
    m_pdhTotalPowerEntry = new DoubleLogEntry(log, "/power/pdh/totalPowerWatts");
    m_pdhTotalEnergyEntry = new DoubleLogEntry(log, "/power/pdh/totalEnergyJoules");
    m_pdhSwitchableChannelEntry = new BooleanLogEntry(log, "/power/pdh/switchableChannelEnabled");

    int pdhChannels = m_powerDistribution != null ? m_powerDistribution.getNumChannels() : 0;
    m_channelCurrentEntries = new DoubleLogEntry[pdhChannels];
    for (int channel = 0; channel < m_channelCurrentEntries.length; channel++) {
      m_channelCurrentEntries[channel] =
          new DoubleLogEntry(log, "/power/pdh/channel" + channel + "CurrentAmps");
    }

    m_intakeSupplyCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/intake/supplyCurrentAmps");
    m_intakeStatorCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/intake/statorCurrentAmps");
    m_intakeVoltageEntry = new DoubleLogEntry(log, "/power/subsystems/intake/motorVoltageVolts");

    m_intakePivotSupplyCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/intakePivot/supplyCurrentAmps");
    m_intakePivotStatorCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/intakePivot/statorCurrentAmps");
    m_intakePivotVoltageEntry =
        new DoubleLogEntry(log, "/power/subsystems/intakePivot/motorVoltageVolts");

    m_indexerFeederSupplyCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/indexer/feederSupplyCurrentAmps");
    m_indexerSpindexerSupplyCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/indexer/spindexerSupplyCurrentAmps");
    m_indexerFeederStatorCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/indexer/feederStatorCurrentAmps");
    m_indexerSpindexerStatorCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/indexer/spindexerStatorCurrentAmps");
    m_indexerFeederVoltageEntry =
        new DoubleLogEntry(log, "/power/subsystems/indexer/feederVoltageVolts");
    m_indexerSpindexerVoltageEntry =
        new DoubleLogEntry(log, "/power/subsystems/indexer/spindexerVoltageVolts");
    m_indexerTotalSupplyCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/indexer/totalSupplyCurrentAmps");

    m_shooterMasterSupplyCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/shooter/masterSupplyCurrentAmps");
    m_shooterSlaveSupplyCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/shooter/slaveSupplyCurrentAmps");
    m_shooterMasterStatorCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/shooter/masterStatorCurrentAmps");
    m_shooterSlaveStatorCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/shooter/slaveStatorCurrentAmps");
    m_shooterMasterVoltageEntry =
        new DoubleLogEntry(log, "/power/subsystems/shooter/masterVoltageVolts");
    m_shooterSlaveVoltageEntry =
        new DoubleLogEntry(log, "/power/subsystems/shooter/slaveVoltageVolts");
    m_shooterTotalSupplyCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/shooter/totalSupplyCurrentAmps");

    m_shooterPivotSupplyCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/shooterPivot/supplyCurrentAmps");
    m_shooterPivotStatorCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/shooterPivot/statorCurrentAmps");
    m_shooterPivotVoltageEntry =
        new DoubleLogEntry(log, "/power/subsystems/shooterPivot/motorVoltageVolts");

    m_mechanismsTotalSupplyCurrentEntry =
        new DoubleLogEntry(log, "/power/subsystems/totalMechanismSupplyCurrentAmps");
    m_diagnosticSummaryEntry = new StringLogEntry(log, "/power/diagnostics/summary");
    m_diagnosticEventEntry = new StringLogEntry(log, "/power/diagnostics/events");

    DataLogManager.log("[PowerDiagnostics] Power diagnostics logger initialized");
    if (m_powerDistribution == null) {
      DataLogManager.log("[PowerDiagnostics] PD telemetry disabled until PD can be read");
      m_pdhTelemetryEnabled = false;
      m_nextPdhRetryTimestampSeconds = 0.0;
    }
  }

  public void logPeriodic() {
    double nowSeconds = RobotController.getFPGATime() / 1_000_000.0;
    if (nowSeconds - m_lastLogTimestampSeconds < LOG_PERIOD_SECONDS) {
      return;
    }
    m_lastLogTimestampSeconds = nowSeconds;

    double batteryVoltage = RobotController.getBatteryVoltage();
    m_batteryVoltageEntry.append(batteryVoltage);
    m_brownoutVoltageEntry.append(RobotController.getBrownoutVoltage());

    logPdhTelemetry(nowSeconds);

    double intakeSupply = m_intake.getSupplyCurrentAmps();
    double intakePivotSupply = m_intakePivot.getSupplyCurrentAmps();
    double feederSupply = 0.0;
    double spindexerSupply = 0.0;
    double shooterMasterSupply = 0.0;
    double shooterSlaveSupply = 0.0;
    double shooterPivotSupply = m_shooterPivot.getSupplyCurrentAmps();

    m_intakeSupplyCurrentEntry.append(intakeSupply);
    m_intakeStatorCurrentEntry.append(m_intake.getStatorCurrentAmps());
    m_intakeVoltageEntry.append(m_intake.getMotorVoltageVolts());

    m_intakePivotSupplyCurrentEntry.append(intakePivotSupply);
    m_intakePivotStatorCurrentEntry.append(m_intakePivot.getStatorCurrentAmps());
    m_intakePivotVoltageEntry.append(m_intakePivot.getMotorVoltageVolts());

    try {
      feederSupply = m_indexer.getFeederSupplyCurrentAmps();
      spindexerSupply = m_indexer.getSpindexerSupplyCurrentAmps();
      m_indexerFeederSupplyCurrentEntry.append(feederSupply);
      m_indexerSpindexerSupplyCurrentEntry.append(spindexerSupply);
      m_indexerFeederStatorCurrentEntry.append(m_indexer.getFeederStatorCurrentAmps());
      m_indexerSpindexerStatorCurrentEntry.append(m_indexer.getSpindexerStatorCurrentAmps());
      m_indexerFeederVoltageEntry.append(m_indexer.getFeederVoltageVolts());
      m_indexerSpindexerVoltageEntry.append(m_indexer.getSpindexerVoltageVolts());
      m_indexerTotalSupplyCurrentEntry.append(feederSupply + spindexerSupply);
    } catch (RuntimeException ex) {
      logFailureRateLimited(
          nowSeconds, "[PowerDiagnostics] Indexer telemetry read failed: " + ex.getMessage());
    }

    try {
      shooterMasterSupply = m_shooter.getMasterSupplyCurrentAmps();
      shooterSlaveSupply = m_shooter.getSlaveSupplyCurrentAmps();
      m_shooterMasterSupplyCurrentEntry.append(shooterMasterSupply);
      m_shooterSlaveSupplyCurrentEntry.append(shooterSlaveSupply);
      m_shooterMasterStatorCurrentEntry.append(m_shooter.getMasterStatorCurrentAmps());
      m_shooterSlaveStatorCurrentEntry.append(m_shooter.getSlaveStatorCurrentAmps());
      m_shooterMasterVoltageEntry.append(m_shooter.getMasterVoltageVolts());
      m_shooterSlaveVoltageEntry.append(m_shooter.getSlaveVoltageVolts());
      m_shooterTotalSupplyCurrentEntry.append(shooterMasterSupply + shooterSlaveSupply);
    } catch (RuntimeException ex) {
      logFailureRateLimited(
          nowSeconds, "[PowerDiagnostics] Shooter telemetry read failed: " + ex.getMessage());
    }

    m_shooterPivotSupplyCurrentEntry.append(shooterPivotSupply);
    m_shooterPivotStatorCurrentEntry.append(m_shooterPivot.getStatorCurrentAmps());
    m_shooterPivotVoltageEntry.append(m_shooterPivot.getMotorVoltageVolts());

    double mechanismsTotalSupplyCurrent = intakeSupply
        + intakePivotSupply
        + feederSupply
        + spindexerSupply
        + shooterMasterSupply
        + shooterSlaveSupply
        + shooterPivotSupply;
    m_mechanismsTotalSupplyCurrentEntry.append(mechanismsTotalSupplyCurrent);

    double[] mechanismCurrents = {
      intakeSupply,
      intakePivotSupply,
      feederSupply,
      spindexerSupply,
      shooterMasterSupply,
      shooterSlaveSupply,
      shooterPivotSupply
    };
    String[] mechanismNames = {
      "intake",
      "intakePivot",
      "indexerFeeder",
      "indexerSpindexer",
      "shooterMaster",
      "shooterSlave",
      "shooterPivot"
    };

    logDiagnosticEvents(
        nowSeconds,
        batteryVoltage,
        mechanismsTotalSupplyCurrent,
        mechanismNames,
        mechanismCurrents);
    logDiagnosticSummary(
        nowSeconds,
        batteryVoltage,
        mechanismsTotalSupplyCurrent,
        mechanismNames,
        mechanismCurrents);

    m_lastMechanismTotalAmps = mechanismsTotalSupplyCurrent;
  }

  private void logPdhTelemetry(double nowSeconds) {
    if (m_powerDistribution == null) {
      return;
    }

    if (!m_pdhTelemetryEnabled && nowSeconds < m_nextPdhRetryTimestampSeconds) {
      return;
    }

    try {
      m_pdhVoltageEntry.append(m_powerDistribution.getVoltage());
      m_pdhTemperatureEntry.append(m_powerDistribution.getTemperature());
      m_pdhTotalCurrentEntry.append(m_powerDistribution.getTotalCurrent());
      m_pdhTotalPowerEntry.append(m_powerDistribution.getTotalPower());
      m_pdhTotalEnergyEntry.append(m_powerDistribution.getTotalEnergy());
      m_pdhSwitchableChannelEntry.append(m_powerDistribution.getSwitchableChannel());

      for (int channel = 0; channel < m_channelCurrentEntries.length; channel++) {
        m_channelCurrentEntries[channel].append(m_powerDistribution.getCurrent(channel));
      }

      if (!m_pdhTelemetryEnabled) {
        DataLogManager.log("[PowerDiagnostics] PD telemetry recovered");
      }
      m_pdhTelemetryEnabled = true;
      m_pdhFailureCount = 0;
      m_lastPdhTotalCurrentAmps = m_powerDistribution.getTotalCurrent();
      m_lastPdhVoltageVolts = m_powerDistribution.getVoltage();
    } catch (RuntimeException ex) {
      m_pdhTelemetryEnabled = false;
      m_pdhFailureCount++;

      double retryDelaySeconds = Math.min(
          PDH_RETRY_MAX_SECONDS, PDH_RETRY_BASE_SECONDS * Math.pow(2.0, m_pdhFailureCount - 1));
      m_nextPdhRetryTimestampSeconds = nowSeconds + retryDelaySeconds;

      logFailureRateLimited(
          nowSeconds,
          "[PowerDiagnostics] PD read failed (retry in "
              + String.format("%.2f", retryDelaySeconds)
              + "s): " + ex.getMessage());
    }
  }

  private void logDiagnosticSummary(
      double nowSeconds,
      double batteryVoltage,
      double mechanismsTotalSupplyCurrent,
      String[] mechanismNames,
      double[] mechanismCurrents) {
    if (nowSeconds - m_lastSummaryTimestampSeconds < SUMMARY_LOG_PERIOD_SECONDS) {
      return;
    }
    m_lastSummaryTimestampSeconds = nowSeconds;

    int[] top = getTopThreeIndices(mechanismCurrents);
    String pdhTotal = Double.isNaN(m_lastPdhTotalCurrentAmps)
        ? "na"
        : String.format("%.1f", m_lastPdhTotalCurrentAmps);
    String pdhVoltage =
        Double.isNaN(m_lastPdhVoltageVolts) ? "na" : String.format("%.2f", m_lastPdhVoltageVolts);

    String flags = "none";
    if (batteryVoltage <= BATTERY_LOW_THRESHOLD_VOLTS) {
      flags = "batteryLow";
    }
    if (mechanismsTotalSupplyCurrent >= HIGH_MECHANISM_CURRENT_THRESHOLD_AMPS) {
      flags = flags.equals("none") ? "highMechCurrent" : flags + "|highMechCurrent";
    }
    if (!m_pdhTelemetryEnabled) {
      flags = flags.equals("none") ? "pdhUnavailable" : flags + "|pdhUnavailable";
    }

    String summary = String.format(
        "t=%.2f,batt=%.2f,mechTotal=%.1f,pdhTotal=%s,pdhVolt=%s,top1=%s:%.1f,top2=%s:%.1f,top3=%s:%.1f,flags=%s",
        nowSeconds,
        batteryVoltage,
        mechanismsTotalSupplyCurrent,
        pdhTotal,
        pdhVoltage,
        mechanismNames[top[0]],
        mechanismCurrents[top[0]],
        mechanismNames[top[1]],
        mechanismCurrents[top[1]],
        mechanismNames[top[2]],
        mechanismCurrents[top[2]],
        flags);
    m_diagnosticSummaryEntry.append(summary);
  }

  private void logDiagnosticEvents(
      double nowSeconds,
      double batteryVoltage,
      double mechanismsTotalSupplyCurrent,
      String[] mechanismNames,
      double[] mechanismCurrents) {
    int[] top = getTopThreeIndices(mechanismCurrents);

    boolean batteryLowNow = batteryVoltage <= BATTERY_LOW_THRESHOLD_VOLTS;
    if (batteryLowNow && !m_batteryLowActive) {
      m_diagnosticEventEntry.append(String.format(
          "t=%.2f,event=batteryLowStart,batt=%.2f,top=%s:%.1f",
          nowSeconds, batteryVoltage, mechanismNames[top[0]], mechanismCurrents[top[0]]));
    } else if (!batteryLowNow && m_batteryLowActive) {
      m_diagnosticEventEntry.append(
          String.format("t=%.2f,event=batteryLowEnd,batt=%.2f", nowSeconds, batteryVoltage));
    }
    m_batteryLowActive = batteryLowNow;

    boolean highCurrentNow = mechanismsTotalSupplyCurrent >= HIGH_MECHANISM_CURRENT_THRESHOLD_AMPS;
    if (highCurrentNow && !m_highCurrentActive) {
      m_diagnosticEventEntry.append(String.format(
          "t=%.2f,event=highMechanismCurrentStart,total=%.1f,top=%s:%.1f",
          nowSeconds,
          mechanismsTotalSupplyCurrent,
          mechanismNames[top[0]],
          mechanismCurrents[top[0]]));
    } else if (!highCurrentNow && m_highCurrentActive) {
      m_diagnosticEventEntry.append(String.format(
          "t=%.2f,event=highMechanismCurrentEnd,total=%.1f",
          nowSeconds, mechanismsTotalSupplyCurrent));
    }
    m_highCurrentActive = highCurrentNow;

    if (!Double.isNaN(m_lastMechanismTotalAmps)) {
      double delta = mechanismsTotalSupplyCurrent - m_lastMechanismTotalAmps;
      if (delta >= CURRENT_SPIKE_THRESHOLD_AMPS) {
        m_diagnosticEventEntry.append(String.format(
            "t=%.2f,event=mechanismCurrentSpike,delta=%.1f,total=%.1f,top=%s:%.1f",
            nowSeconds,
            delta,
            mechanismsTotalSupplyCurrent,
            mechanismNames[top[0]],
            mechanismCurrents[top[0]]));
      }
    }
  }

  private int[] getTopThreeIndices(double[] values) {
    int first = 0;
    int second = values.length > 1 ? 1 : 0;
    int third = values.length > 2 ? 2 : second;

    if (values[second] > values[first]) {
      int temp = first;
      first = second;
      second = temp;
    }
    if (values[third] > values[second]) {
      int temp = second;
      second = third;
      third = temp;
      if (values[second] > values[first]) {
        temp = first;
        first = second;
        second = temp;
      }
    }

    for (int i = 3; i < values.length; i++) {
      if (values[i] > values[first]) {
        third = second;
        second = first;
        first = i;
      } else if (values[i] > values[second]) {
        third = second;
        second = i;
      } else if (values[i] > values[third]) {
        third = i;
      }
    }

    return new int[] {first, second, third};
  }

  private void logFailureRateLimited(double nowSeconds, String message) {
    if (nowSeconds - m_lastFailureLogTimestampSeconds < FAILURE_LOG_PERIOD_SECONDS) {
      return;
    }
    m_lastFailureLogTimestampSeconds = nowSeconds;
    DataLogManager.log(message);
  }
}
