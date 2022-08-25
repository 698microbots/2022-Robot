// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;

public class FlyWheelSubsystem extends SubsystemBase {
  /** Creates a new FlyWheelSubsystem. */
  private final TalonFX flyWheelMotor = new TalonFX(Constants.flyWheelMotorID);

  public FlyWheelSubsystem() {}

  public void runFlywheel(double input) {
    flyWheelMotor.set(ControlMode.PercentOutput, input);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
