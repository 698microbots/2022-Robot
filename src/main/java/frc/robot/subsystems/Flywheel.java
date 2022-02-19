// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
/*




THIS IS ONLY TEST CODE
IT IS NOT FINAL




*/
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.subsystems.*;
import io.github.pseudoresonance.pixy2api.Pixy2;
import edu.wpi.first.wpilibj.*;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  public Flywheel() {}
  private static final TalonFX turretWheel = new TalonFX(Constants.flywheelID);
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void setFlywheelSpeed()
  {
    turretWheel.set(ControlMode.PercentOutput, 1);//change the speed value for different rpm
  }

  public static void stopFlywheel()
  {
      turretWheel.set(ControlMode.PercentOutput, 0);
  }
}

