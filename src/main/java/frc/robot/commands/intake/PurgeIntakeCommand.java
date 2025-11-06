// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PurgeIntakeCommand extends Command {
  public final IntakeSubsystem intake;
  public final IndexerSubsystem indexer;
  /** Creates a new PurgeIntakeCommand. */
  public PurgeIntakeCommand(IntakeSubsystem mIntake, IndexerSubsystem mIndexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = mIntake;
    indexer = mIndexer;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.purgeIntake();
    indexer.purgeIndexer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.purgeIntake();
    indexer.purgeIndexer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
