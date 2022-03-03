// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  /** Creates a new Index. */
  public static Constants constant;
  private static CANSparkMax indexMotorTop; 
  private static CANSparkMax indexMotorBottom;  




  public Index() {
    indexMotorTop = new CANSparkMax(constant.IndexMotorIDTop, MotorType.kBrushless);
    indexMotorBottom = new CANSparkMax(constant.IndexMotorIDBottom, MotorType.kBrushless);
  }

  public static void IndexOn () {

    indexMotorBottom.set(constant.indexMotorSpeedBottom);
    indexMotorTop.set(constant.indexMotorSpeedTop);
  }

  public static void IndexOff () {
    indexMotorTop.set(0);
    indexMotorBottom.set(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
