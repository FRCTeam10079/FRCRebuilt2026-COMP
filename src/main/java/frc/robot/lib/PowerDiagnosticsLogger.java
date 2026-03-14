package frc.robot.lib;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
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

    private final PowerDistribution m_powerDistribution = new PowerDistribution();
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

    private double m_lastLogTimestampSeconds = Double.NEGATIVE_INFINITY;

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

        DataLog log = DataLogManager.getLog();

        m_batteryVoltageEntry = new DoubleLogEntry(log, "/power/robot/batteryVoltageVolts");
        m_brownoutVoltageEntry = new DoubleLogEntry(log, "/power/robot/brownoutVoltageVolts");
        m_pdhVoltageEntry = new DoubleLogEntry(log, "/power/pdh/voltageVolts");
        m_pdhTemperatureEntry = new DoubleLogEntry(log, "/power/pdh/temperatureCelsius");
        m_pdhTotalCurrentEntry = new DoubleLogEntry(log, "/power/pdh/totalCurrentAmps");
        m_pdhTotalPowerEntry = new DoubleLogEntry(log, "/power/pdh/totalPowerWatts");
        m_pdhTotalEnergyEntry = new DoubleLogEntry(log, "/power/pdh/totalEnergyJoules");
        m_pdhSwitchableChannelEntry = new BooleanLogEntry(log, "/power/pdh/switchableChannelEnabled");

        m_channelCurrentEntries = new DoubleLogEntry[m_powerDistribution.getNumChannels()];
        for (int channel = 0; channel < m_channelCurrentEntries.length; channel++) {
            m_channelCurrentEntries[channel] = new DoubleLogEntry(log, "/power/pdh/channel" + channel + "CurrentAmps");
        }

        m_intakeSupplyCurrentEntry = new DoubleLogEntry(log, "/power/subsystems/intake/supplyCurrentAmps");
        m_intakeStatorCurrentEntry = new DoubleLogEntry(log, "/power/subsystems/intake/statorCurrentAmps");
        m_intakeVoltageEntry = new DoubleLogEntry(log, "/power/subsystems/intake/motorVoltageVolts");

        m_intakePivotSupplyCurrentEntry = new DoubleLogEntry(log, "/power/subsystems/intakePivot/supplyCurrentAmps");
        m_intakePivotStatorCurrentEntry = new DoubleLogEntry(log, "/power/subsystems/intakePivot/statorCurrentAmps");
        m_intakePivotVoltageEntry = new DoubleLogEntry(log, "/power/subsystems/intakePivot/motorVoltageVolts");

        m_indexerFeederSupplyCurrentEntry = new DoubleLogEntry(log,
                "/power/subsystems/indexer/feederSupplyCurrentAmps");
        m_indexerSpindexerSupplyCurrentEntry = new DoubleLogEntry(log,
                "/power/subsystems/indexer/spindexerSupplyCurrentAmps");
        m_indexerFeederStatorCurrentEntry = new DoubleLogEntry(log,
                "/power/subsystems/indexer/feederStatorCurrentAmps");
        m_indexerSpindexerStatorCurrentEntry = new DoubleLogEntry(log,
                "/power/subsystems/indexer/spindexerStatorCurrentAmps");
        m_indexerFeederVoltageEntry = new DoubleLogEntry(log, "/power/subsystems/indexer/feederVoltageVolts");
        m_indexerSpindexerVoltageEntry = new DoubleLogEntry(log, "/power/subsystems/indexer/spindexerVoltageVolts");
        m_indexerTotalSupplyCurrentEntry = new DoubleLogEntry(log, "/power/subsystems/indexer/totalSupplyCurrentAmps");

        m_shooterMasterSupplyCurrentEntry = new DoubleLogEntry(log,
                "/power/subsystems/shooter/masterSupplyCurrentAmps");
        m_shooterSlaveSupplyCurrentEntry = new DoubleLogEntry(log, "/power/subsystems/shooter/slaveSupplyCurrentAmps");
        m_shooterMasterStatorCurrentEntry = new DoubleLogEntry(log,
                "/power/subsystems/shooter/masterStatorCurrentAmps");
        m_shooterSlaveStatorCurrentEntry = new DoubleLogEntry(log, "/power/subsystems/shooter/slaveStatorCurrentAmps");
        m_shooterMasterVoltageEntry = new DoubleLogEntry(log, "/power/subsystems/shooter/masterVoltageVolts");
        m_shooterSlaveVoltageEntry = new DoubleLogEntry(log, "/power/subsystems/shooter/slaveVoltageVolts");
        m_shooterTotalSupplyCurrentEntry = new DoubleLogEntry(log, "/power/subsystems/shooter/totalSupplyCurrentAmps");

        m_shooterPivotSupplyCurrentEntry = new DoubleLogEntry(log, "/power/subsystems/shooterPivot/supplyCurrentAmps");
        m_shooterPivotStatorCurrentEntry = new DoubleLogEntry(log, "/power/subsystems/shooterPivot/statorCurrentAmps");
        m_shooterPivotVoltageEntry = new DoubleLogEntry(log, "/power/subsystems/shooterPivot/motorVoltageVolts");

        m_mechanismsTotalSupplyCurrentEntry = new DoubleLogEntry(log,
                "/power/subsystems/totalMechanismSupplyCurrentAmps");

        DataLogManager.log("[PowerDiagnostics] Power diagnostics logger initialized");
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

        m_pdhVoltageEntry.append(m_powerDistribution.getVoltage());
        m_pdhTemperatureEntry.append(m_powerDistribution.getTemperature());
        m_pdhTotalCurrentEntry.append(m_powerDistribution.getTotalCurrent());
        m_pdhTotalPowerEntry.append(m_powerDistribution.getTotalPower());
        m_pdhTotalEnergyEntry.append(m_powerDistribution.getTotalEnergy());
        m_pdhSwitchableChannelEntry.append(m_powerDistribution.getSwitchableChannel());

        for (int channel = 0; channel < m_channelCurrentEntries.length; channel++) {
            m_channelCurrentEntries[channel].append(m_powerDistribution.getCurrent(channel));
        }

        double intakeSupply = m_intake.getSupplyCurrentAmps();
        double intakePivotSupply = m_intakePivot.getSupplyCurrentAmps();
        double feederSupply = m_indexer.getFeederSupplyCurrentAmps();
        double spindexerSupply = m_indexer.getSpindexerSupplyCurrentAmps();
        double shooterMasterSupply = m_shooter.getMasterSupplyCurrentAmps();
        double shooterSlaveSupply = m_shooter.getSlaveSupplyCurrentAmps();
        double shooterPivotSupply = m_shooterPivot.getSupplyCurrentAmps();

        m_intakeSupplyCurrentEntry.append(intakeSupply);
        m_intakeStatorCurrentEntry.append(m_intake.getStatorCurrentAmps());
        m_intakeVoltageEntry.append(m_intake.getMotorVoltageVolts());

        m_intakePivotSupplyCurrentEntry.append(intakePivotSupply);
        m_intakePivotStatorCurrentEntry.append(m_intakePivot.getStatorCurrentAmps());
        m_intakePivotVoltageEntry.append(m_intakePivot.getMotorVoltageVolts());

        m_indexerFeederSupplyCurrentEntry.append(feederSupply);
        m_indexerSpindexerSupplyCurrentEntry.append(spindexerSupply);
        m_indexerFeederStatorCurrentEntry.append(m_indexer.getFeederStatorCurrentAmps());
        m_indexerSpindexerStatorCurrentEntry.append(m_indexer.getSpindexerStatorCurrentAmps());
        m_indexerFeederVoltageEntry.append(m_indexer.getFeederVoltageVolts());
        m_indexerSpindexerVoltageEntry.append(m_indexer.getSpindexerVoltageVolts());
        m_indexerTotalSupplyCurrentEntry.append(feederSupply + spindexerSupply);

        m_shooterMasterSupplyCurrentEntry.append(shooterMasterSupply);
        m_shooterSlaveSupplyCurrentEntry.append(shooterSlaveSupply);
        m_shooterMasterStatorCurrentEntry.append(m_shooter.getMasterStatorCurrentAmps());
        m_shooterSlaveStatorCurrentEntry.append(m_shooter.getSlaveStatorCurrentAmps());
        m_shooterMasterVoltageEntry.append(m_shooter.getMasterVoltageVolts());
        m_shooterSlaveVoltageEntry.append(m_shooter.getSlaveVoltageVolts());
        m_shooterTotalSupplyCurrentEntry.append(shooterMasterSupply + shooterSlaveSupply);

        m_shooterPivotSupplyCurrentEntry.append(shooterPivotSupply);
        m_shooterPivotStatorCurrentEntry.append(m_shooterPivot.getStatorCurrentAmps());
        m_shooterPivotVoltageEntry.append(m_shooterPivot.getMotorVoltageVolts());

        m_mechanismsTotalSupplyCurrentEntry.append(intakeSupply
                + intakePivotSupply
                + feederSupply
                + spindexerSupply
                + shooterMasterSupply
                + shooterSlaveSupply
                + shooterPivotSupply);
    }
}
