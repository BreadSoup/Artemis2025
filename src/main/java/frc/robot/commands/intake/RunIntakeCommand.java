// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntakeCommand extends Command {
  public final IntakeSubsystem intake;
  public final IndexerSubsystem indexer;
  /** Creates a new IntakeCommand. */
  public RunIntakeCommand(IntakeSubsystem mIntake, IndexerSubsystem mIndexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = mIntake;
    addRequirements(mIntake);

    indexer = mIndexer;
    addRequirements(mIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.runIntake();
    intake.lowerIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake();
    indexer.runIndexer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    intake.raiseIntake();
    indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
