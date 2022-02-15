// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.subsystems.*;
import io.github.pseudoresonance.pixy2api.Pixy2;
import edu.wpi.first.wpilibj.*;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static CANSparkMax intakeMotor;  
  private static PixyCam pixy;
  
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.deviceIdIntake, CANSparkMax.MotorType.kBrushless);
    pixy = new PixyCam();

  }

  public static void intputBall()
  {
    intakeMotor.set(Constants.intakeMotorSpeed);
  }


  public static void stopMotor()
  {
    if(getElectricCurrent() >= Constants.ampSpike)
      intakeMotor.set(0);
    
  }

  public static void outputBall()
  {
    
    intakeMotor.set(-Constants.intakeMotorSpeed);
  }

  public static double getElectricCurrent()
  {
    return intakeMotor.getOutputCurrent();
  }

  public static int determineClosestBall()
  {
    int blueXCoord = pixy.getBlueXcordinate();
    int blueYCoord = pixy.getBlueYcordinate();
    int redXCoord = pixy.getRedXcordinate();
    int redYCoord = pixy.getRedYcordinate();
    double redXDist = redXCoord - Constants.closestX;
    double redYDist = redYCoord - Constants.closestY;
    double blueXDist = blueXCoord - Constants.closestX;
    double blueYDist = blueYCoord - Constants.closestY;
    double redDist = Math.sqrt(Math.pow(redXDist, 2) + Math.pow(redYDist, 2));
    double blueDist = Math.sqrt(Math.pow(blueXDist, 2) + Math.pow(blueYDist, 2));
    if(redDist>blueDist)
    {
      return 1;
    }
    return 2;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
