// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoTurn extends CommandBase {
  /** Creates a new AutoTurn. */
  private final DriveTrainSubsystem driveTrain;
  private final double target;
  private final Supplier<Float> navXInput;
  private int counter;

  public AutoTurn(DriveTrainSubsystem driveTrain, double target, Supplier<Float> navXInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.target = target;
    counter = 0;
    this.navXInput = navXInput;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Automatic turning has started!");
    driveTrain.setTurnTarget(driveTrain.getTurnTarget() + target);
    //need to check if reset encoders is still neccessary.
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //calculate turn speed
    double sensorInput = (double) navXInput.get();
    driveTrain.PIDturn(sensorInput);

    //set turn speed
    driveTrain.setRightSpeed(driveTrain.getTurnOutput());
    driveTrain.setLeftSpeed(-driveTrain.getTurnOutput());

    //Increment the counter when error is small enough
    if(driveTrain.getPIDTurnError() < 0.1){
      counter++;
    }else{
      counter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Automatic turning has ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if the counter is big enough, this method returns true causing the command to end.
    if(counter > 10){
      return true;
    }else{
      counter = 0;
    return false;
    }
  }
}
