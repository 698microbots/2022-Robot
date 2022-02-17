// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PixyCamSubsystem;

public class AutoTrackingRedBall extends CommandBase {
  /** Creates a new AutoTrackingRedBall. */
  private final DriveTrainSubsystem driveTrain;
  private final PixyCamSubsystem pixy2;
  private final Supplier<Double> navXAngleSensor;
  private int counter;
  
  public AutoTrackingRedBall(DriveTrainSubsystem driveTrain, PixyCamSubsystem pixy2, Supplier<Double> navXAngleSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.pixy2 = pixy2;
    this.navXAngleSensor = navXAngleSensor;
    counter = 0;
    addRequirements(driveTrain);
    addRequirements(pixy2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //checks if the closest or second closest block is a red ball.
    if(pixy2.getBlockSignature(0) == 1){//sets the turn target to closest block if it is red
      driveTrain.setTurnTarget(driveTrain.getTurnTarget()+pixy2.getBlockXangle(0));
    }else if(pixy2.getBlockSignature(1) == 1){//sets turn target to second farthest block iof it is red
      driveTrain.setTurnTarget(driveTrain.getTurnTarget()+pixy2.getBlockXangle(1));
    }else{
      driveTrain.setTurnTarget(0.0);
    }

    double sensorInput = navXAngleSensor.get();
    driveTrain.PIDturn(sensorInput);

    //Increment the counter when error is small enough
    if(driveTrain.getPIDTurnError() < 0.1){
      counter++;
    }else{
      counter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter > 10){
      return true;
    }else{
      counter = 0;
      return false;
    }
  }
}
