// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoTimedDrive extends CommandBase {
  /** Creates a new AutoTimedDrive. */
  private final DriveTrainSubsystem driveTrain;
  private int counter;
  private double time;
  private final double speed;
  public AutoTimedDrive(DriveTrainSubsystem driveTrain, double time, double speed) {
    this.driveTrain = driveTrain;
    this.time = time;
    this.speed = -speed;
    counter = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.setRightSpeed(speed);
    driveTrain.setLeftSpeed(speed);
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLeftSpeed(0);
    driveTrain.setRightSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter>=time/20){
      return true;
    }else{
      return false;
    }
  }
}
