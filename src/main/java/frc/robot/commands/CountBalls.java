// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallCounterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class CountBalls extends CommandBase {
  /** Creates a new CountBalls. */
  private BallCounterSubsystem ballCounter;
  private IndexerSubsystem index;
  private boolean latchTop;
  private boolean latchBottom;
  public CountBalls(BallCounterSubsystem ballCounter, IndexerSubsystem index) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ballCounter = ballCounter;
    this.index = index;
    addRequirements(this.ballCounter);
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    latchBottom = true;
    latchTop = true;
    RobotContainer.ballCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("" + ballCounter.topSensorStatus());
    if (!ballCounter.bottomSensorStatus()){
      if (!latchBottom){
        if(!index.isReversed()){
          RobotContainer.ballCount++;
          System.out.println("Ball added: " + RobotContainer.ballCount);
        } else{
          RobotContainer.ballCount--;
          System.out.println("Ball removed: " + RobotContainer.ballCount);
        }
        
        latchBottom = true;
      }
    }else {
      latchBottom = false;
    }

    if (!ballCounter.topSensorStatus()){
        if (!latchTop){
          RobotContainer.ballCount--;
          System.out.println("Ball removed: " + RobotContainer.ballCount);
          latchTop = true;
        }
      }else {
        latchTop = false;
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.ballCount = 0;
    latchTop = false;
    latchBottom = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
