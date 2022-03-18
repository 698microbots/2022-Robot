// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class BallCounterSubsystem extends SubsystemBase {
  /** Creates a new BallCounter. **/
  
  private DigitalInput bottomPhotovalticSensor = new DigitalInput(Constants.bottomPhotovalticID); // class for port 1
  private AnalogInput topPhotovolaticSenor = new AnalogInput(0); // class for port 2

  
  public BallCounterSubsystem() {
  
  }

  public boolean bottomSensorStatus() 
  {
    return bottomPhotovalticSensor.get();
  }

  public boolean topSensorStatus()
  {
    if (topPhotovolaticSenor.getVoltage() > 3) 
      return false;
    else
      return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
