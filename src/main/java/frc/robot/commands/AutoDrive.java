// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  private final DriveTrainSubsystem driveTrain;
  private final double distance;
  private final Supplier <Float> navXInput;
  private int counter;

  public AutoDrive(DriveTrainSubsystem driveTrain, double distance, Supplier <Float> navXInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.distance = distance;
    this.navXInput = navXInput;
    counter = 0;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Auto driving has started!");
    // driveTrain.PIDdrive(-distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //calculate motor speeds
    float sensorInput = navXInput.get();
    driveTrain.PIDdrive(sensorInput);

    //set motor speeds
    driveTrain.setRightSpeed(driveTrain.getDriveOutput());
    driveTrain.setLeftSpeed(driveTrain.getDriveOutput());

    //check if target is met
    if(driveTrain.getDriveError()<0.1){
      counter++;
    }else{
      counter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Auto driving has ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(counter > 10){
      return true;
    }else{
      counter = 0;
    }
    return false;
  }
}
