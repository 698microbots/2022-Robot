// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystems extends SubsystemBase {
  /** Creates a new Vision. */
  private NetworkTable limeLight;
  private NetworkTableEntry V_angle;
  private NetworkTableEntry H_angle;
  private double zDistance;
  private double xDistance;

  public VisionSubsystems() {
    limeLight = NetworkTableInstance.getDefault().getTable("limelight");
    V_angle = limeLight.getEntry("ts");
    H_angle = limeLight.getEntry("tx");
    zDistance = -1;//this value is for if there's an error, makes sense that distance will never be negative
    xDistance = -1;//the distance in the x direction offset from center of robot.
  }

  //methods
  public double calculateZdistance(){
    zDistance = Constants.goalHeight/(Math.tan(Math.toRadians(getV_angle()+25)));
    return zDistance;
  }

  public double calculateXdistance(){
    xDistance = calculateZdistance()*Math.tan(Math.toRadians(getH_angle()));
    return xDistance;
  }
  //getters
  public double getV_angle(){
    return V_angle.getDouble(0.0);
  }

  public double getH_angle(){
    return H_angle.getDouble(0.00);
  }

  public double getZdistance(){//kind of redundant, but ok?
    return zDistance;
  }

  public double getXdistance(){//same here
    return xDistance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
