// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
  /** Creates a new Wait. */
  private int millis;
  private int counter; 
  public Wait(int millis) { //?? creating object, out of constructor, passing it through constructor, giving it a "new name/redfine it" so it can be used in methods
    this.millis = millis;
    counter = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter >= (millis/20)){
      return true;
    }else{
      return false;
    }
    
  }
}
