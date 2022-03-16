// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallCounter;

public class CountBalls extends CommandBase {
  /** Creates a new CountBalls. */
  private BallCounter ballCounter;
  private boolean latchTop;
  private boolean latchBottom;
  public CountBalls(BallCounter ballCounter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ballCounter = ballCounter;
    addRequirements(this.ballCounter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    latchBottom = false;
    latchTop = false;
    RobotContainer.ballCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (ballCounter.bottomSensorStatus()){
      if (!latchBottom){
        RobotContainer.ballCount++;
        System.out.println("Ball added: " + RobotContainer.ballCount);
        latchBottom = true;
      }
    }else {
      latchBottom = false;
    }

    if (ballCounter.topSensorStatus()){
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
