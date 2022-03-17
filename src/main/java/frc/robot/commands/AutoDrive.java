// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  private final DriveTrainSubsystem driveTrain;
  private final double distance;
  private int counter;
  private double ltime;
  private double limit;

  public AutoDrive(DriveTrainSubsystem driveTrain, double distance, double ltime, double limit) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.distance = distance;
    this.ltime = ltime;
    this.limit = limit;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Auto driving has started!");
    SmartDashboard.putString("AutoDrive Stat", "Auto driving has started!");
    driveTrain.setDriveTarget(-distance*2048/2.75);
    counter = 0;
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //calculate motor speeds
    driveTrain.PIDdrive(driveTrain.getEncoderPosition(), limit);

    //set motor speeds
    driveTrain.setLeftSpeed(driveTrain.getDriveOutput());
    driveTrain.setRightSpeed(driveTrain.getDriveOutput());

    //driveTrain.resetEncoders();
    //check if target is met
    
    // if(Math.abs(driveTrain.getDriveError()) < 10000){
    //   counter++;
    // }else{
    //   counter = 0;
    // }
    counter++;
    SmartDashboard.putNumber("Counter: ", counter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLeftSpeed(0.0);
    driveTrain.setRightSpeed(0.0);
    System.out.println("Auto driving has ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(counter >= ltime/20){
      return true;
    }else{
      return false;
    }
    
   }
}
