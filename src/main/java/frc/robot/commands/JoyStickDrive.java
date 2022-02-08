/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class JoyStickDrive extends CommandBase {
  /**
   * Creates a new JoyStickDrive.
   */

  public JoyStickDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }
//left and right stick inputs from controller

public static double leftStick;
public static double rightStick;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  


  public void execute() 
  {
    leftStick = Math.pow(Robot.oi.Xbox.getRawAxis(Constants.XBOX_L_YAXIS),3.0);
    rightStick = Math.pow(Robot.oi.Xbox.getRawAxis(Constants.XBOX_R_XAXIS),3.0);

    //deadband
    if(Math.abs(leftStick) < 0.001){
      leftStick = 0;
    }
    if(Math.abs(rightStick) < 0.001){
      rightStick = 0;
    }
    
    //config motor speed based on controller input
    Robot.drive.leftSpeed(leftStick-rightStick/2);
    Robot.drive.rightSpeed(leftStick +rightStick/2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
