// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants;

public class JoyStickDrive extends CommandBase {
  /** Creates a new JoyStickDrive. */
  private final DriveTrainSubsystem driveTrain;
  private final Supplier<Double> rightStickFunction, leftStickFunction;
  private final Supplier<Boolean> startPress;
  public JoyStickDrive(DriveTrainSubsystem driveTrain, Supplier<Double> rightStick, Supplier <Double> leftStick, Supplier<Boolean> startPress) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    rightStickFunction = rightStick;
    leftStickFunction = leftStick;
    this.startPress = startPress;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("JoyStickDrive has started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightStick = rightStickFunction.get();
    double leftStick = leftStickFunction.get();
    rightStick = Math.pow(rightStick, 3.0);
    leftStick = Math.pow(leftStick, 3.0);

    //deadband
    // if(Math.abs(rightStick) < 0.05){
    //   rightStick = 0;
    // }
    // if(Math.abs(leftStick) < 0.05){
    //   leftStick = 0;
    // }

    //set the motors using driveTrain subsystem to correct speeds

      if (startPress.get() == true){
      driveTrain.setRightSpeed((rightStick + leftStick/1.9) * .65);
      driveTrain.setLeftSpeed((rightStick - leftStick/1.9) * .65);
      } else {
      driveTrain.setRightSpeed((rightStick + leftStick/1.9) * .01);
      driveTrain.setLeftSpeed((rightStick - leftStick/1.9) * .01);
      }

    
    //reset encoders for 
    // driveTrain.resetEncoders();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setRightSpeed(0);
    driveTrain.setLeftSpeed(0);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
