// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private NetworkTable limeLight;
  private NetworkTableEntry V_angle;
  private NetworkTableEntry H_angle;
  private double distance;

  public Vision() {
    limeLight = NetworkTableInstance.getDefault().getTable("limelight");
    V_angle = limeLight.getEntry("ts");
    H_angle = limeLight.getEntry("tx");

  }

  public double getV_angle(){
    return V_angle.getDouble(0.0);
  }

  public double getH_angle(){
    return H_angle.getDouble(0.00);
  }

  public double findH_distance(){
    double angle = Math.toRadians(getV_angle());
    return Constants.VertDist / (Math.abs(Math.tan(angle)));

    // TODO: fix VertDist to be height - camera height
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
