// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeedCommand extends Command {
  /** Creates a new FeedCommand. */
  public final IndexerSubsystem indexer;
  public FeedCommand(IndexerSubsystem mIndexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    indexer = mIndexer;
    addRequirements(mIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.ShootIndexer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.ShootIndexer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexer();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
