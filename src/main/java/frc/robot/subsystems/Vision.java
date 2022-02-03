// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private NetworkTable limelight;
  private NetworkTableEntry vAngle;
  private NetworkTableEntry hAngle;
  
  public Vision() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    vAngle = limelight.getEntry("ty");
    hAngle = limelight.getEntry("tx");
  }

  public double getVAngle() {
      return vAngle.getDouble(0.0);
  }

  public double getHAngle(){
    return hAngle.getDouble(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
