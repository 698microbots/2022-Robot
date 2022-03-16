// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax intakeMotor;  
  
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.deviceIdIntake, CANSparkMax.MotorType.kBrushless);
  }

  public void intputBall()
  {
    intakeMotor.set(Constants.intakeMotorSpeed);
  }


  public void stopMotor()
  {
    if(getElectricCurrent() >= Constants.ampSpike)
      intakeMotor.set(0);
    
  }

  public void outputBall()
  {
    
    intakeMotor.set(-Constants.intakeMotorSpeed);
  }

  public double getElectricCurrent()
  {
    return intakeMotor.getOutputCurrent();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
