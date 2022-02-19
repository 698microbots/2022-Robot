// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallCounter;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PixyCamSubsystem;

public class AutoRedBallDriver extends CommandBase {
  /** Creates a new AutoRedBallDriver. */
  private final DriveTrainSubsystem driveTrain;
  private final PixyCamSubsystem pixy2;
  private final Supplier<Double> navXangle;
  private final BallCounter ballCounter;

  public AutoRedBallDriver(DriveTrainSubsystem driveTrain, PixyCamSubsystem pixy2, BallCounter ballCounter, Supplier<Double> navXangle) {
    this.driveTrain = driveTrain;
    this.pixy2 = pixy2;
    this.navXangle = navXangle;
    this.ballCounter = ballCounter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    addRequirements(pixy2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //checks if the closest or second closest block is a red ball.
    if(pixy2.getBlockSignature(0) == 1){//sets the turn target to closest block if it is red
      driveTrain.setTurnTarget(navXangle.get()+pixy2.getBlockXangle(0));
    }else if(pixy2.getBlockSignature(1) == 1){//sets turn target to second farthest block iof it is red
      driveTrain.setTurnTarget(navXangle.get()+pixy2.getBlockXangle(1));
    }else{
      driveTrain.setTurnTarget(0.0);
    }
    driveTrain.PIDturn(navXangle.get());//calculate turn speed
    if(pixy2.getBlockCount() > 0){
      driveTrain.setRightSpeed(0.7 + driveTrain.getTurnOutput()/Constants.turnAggressiveness);
      driveTrain.setLeftSpeed(0.7 - driveTrain.getTurnOutput()/Constants.turnAggressiveness);  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ballCounter.getBalls()>0){
      return true;
    }else{
      return false;
    }
  }
}
