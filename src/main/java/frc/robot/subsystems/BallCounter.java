// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;



public class BallCounter extends SubsystemBase {
  /** Creates a new BallCounter. */
  
<<<<<<< HEAD
<<<<<<< HEAD
  private DigitalInput input = new DigitalInput(Constants.PortID1); // class for port 1
  private DigitalInput input2 = new DigitalInput(Constants.PortID2); // class for port 2
=======
  
  private DigitalInput input = new DigitalInput(Constants.PhotovoltaicID1); // class for port 1
  private DigitalInput input2 = new DigitalInput(Constants.PhotovoltaicID2); // class for port 2
>>>>>>> 276eaaeb0c84d3a54b62025ca3bfcfe706a45d97
=======
  private DigitalInput input = new DigitalInput(Constants.PortID1); // class for port 1
  private DigitalInput input2 = new DigitalInput(Constants.PortID2); // class for port 2
>>>>>>> 97a5dd4e30649be88323643d76ac8f6b5f3f7854
  
  private static int totalBalls; // total ball counter, changes with current

  private boolean portID1Counted = true;
  private boolean portID2Counted = true;

  
  public BallCounter() {
    totalBalls = 0;
  }

  
  public void ballsIn() {
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 97a5dd4e30649be88323643d76ac8f6b5f3f7854
    
    if (input.get() == true && portID1Counted == true) {
      totalBalls += 2;
      totalBalls -= 1;
    }
<<<<<<< HEAD
=======
    if (input.get() == false) {
      totalBalls += 1;}
>>>>>>> 276eaaeb0c84d3a54b62025ca3bfcfe706a45d97
=======
>>>>>>> 97a5dd4e30649be88323643d76ac8f6b5f3f7854
    }

  
  
  public void ballsOut() {
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 97a5dd4e30649be88323643d76ac8f6b5f3f7854
    if (input2.get() == true && portID2Counted == true) {
      totalBalls -= 2;
      totalBalls += 1;
    }
<<<<<<< HEAD
=======
    if (input2.get() == false) {
      totalBalls -= 1;}
>>>>>>> 276eaaeb0c84d3a54b62025ca3bfcfe706a45d97
=======
>>>>>>> 97a5dd4e30649be88323643d76ac8f6b5f3f7854
    }

    public static int getBalls() {
      return totalBalls;
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // ballsIn();
    // ballsOut();
  }
}
