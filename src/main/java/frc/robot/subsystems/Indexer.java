// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final CANSparkMax upperSparkMax;
  private final CANSparkMax lowerSparkMax;

  public Indexer() {
    this.upperSparkMax = new CANSparkMax(Constants.upperIndexerID, MotorType.kBrushless);
    this.lowerSparkMax = new CANSparkMax(Constants.lowerIndexerID, MotorType.kBrushless);
  }

  public void runLowerIndexer(double speed){
    lowerSparkMax.set(speed);
  }

  public void runUpperIndexer(double speed){
    upperSparkMax.set(speed);
  }

  public void stopIndexer(){
    lowerSparkMax.set(0.0);
    upperSparkMax.set(0.0);
  }
  
  public void stopLowerIndexer(){
    lowerSparkMax.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
