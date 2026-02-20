// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/** Command that runs the indexer (feeder + spindexer) at specified RPMs while held. */
public class RunIndexer extends Command {

  private final IndexerSubsystem m_indexer;
  private final double m_feederRPM;
  private final double m_spindexerRPM;

  /**
   * Runs the indexer with specific speeds for both motors.
   *
   * @param indexer The subsystem
   * @param feederRPM Target RPM for the fast top roller
   * @param spindexerRPM Target RPM for the floor/star wheel
   */
  public RunIndexer(IndexerSubsystem indexer, double feederRPM, double spindexerRPM) {
    m_indexer = indexer;
    m_feederRPM = feederRPM;
    m_spindexerRPM = spindexerRPM;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    m_indexer.setSpeeds(m_feederRPM, m_spindexerRPM);
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
