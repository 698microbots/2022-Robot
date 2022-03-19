// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallCounterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class AutoIndexHold extends CommandBase {
  /** Creates a new IndexHold. */
  private final IndexerSubsystem index;
  private final BallCounterSubsystem ballCounter;
  private int counter;
  public AutoIndexHold(IndexerSubsystem index, BallCounterSubsystem ballCounter) {
    this.index = index;
    this.ballCounter = ballCounter;
    counter = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(index);
    addRequirements(ballCounter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    index.runUpperIndexer(-0.3);
    index.runLowerIndexer(Constants.indexMotorSpeedBottom);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if(counter >= 1500/20){
        return true;
      }else{
        return false;
      }
  }
}
