// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class BallCounter extends SubsystemBase {
  /** Creates a new BallCounter. **/
  
  private DigitalInput input1 = new DigitalInput(Constants.PortID1); // class for port 1
  private DigitalInput input2 = new DigitalInput(Constants.PortID2); // class for port 2
  private int totalBalls; // total ball counter, changes with current
  private boolean latch1;
  private boolean latch2;
  // private boolean portID1Counted = true;
  // private boolean portID2Counted = true;

  
  public BallCounter() {
    totalBalls = 0;
    latch1 = false;
    latch2 = false;
  }

  public void ballsIn() 
  {
    if (input1.get() == true) 
    {
      if(!latch1)
      {
        totalBalls ++;
        latch1 = true;
        }
    } 
    else 
    {
      latch1 = false;
    }
  }

  public void ballsOut()
  {
    if (input2.get() == true) 
    {
      if(!latch2)
      {
        totalBalls --;
        latch2 = true;
        }
    } 
    else 
    {
      latch2 = false;
    }
  }

  public int getBalls() {
      return totalBalls;
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ballsIn();
    ballsOut();
  }
}
