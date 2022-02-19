// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final TalonFX FrontRight = new TalonFX(Constants.FrontRightID);
  private final TalonFX FrontLeft = new TalonFX(Constants.FrontLeftID);
  private final TalonFX BackRight = new TalonFX(Constants.BackRightID);
  private final TalonFX BackLeft = new TalonFX(Constants.BackLeftID);

  //PIDturn variables
  private double turnTarget;
  private double turnError;
  private double turnPrevError;
  private double turnP;
  private double turnI;
  private double turnD;
  private double turnOutput;
  

  //PIDdrive variables
  private double driveTarget;
  private double driveError;
  private double drivePrevError;
  private double driveP;
  private double driveI;
  private double driveD;
  private double driveOutput;


//Constructors

  public DriveTrainSubsystem() {
    //initialize all neccessary variables and sensors.
    FrontRight.setInverted(false);
    FrontLeft.setInverted(true);
    BackRight.setInverted(false);
    BackLeft.setInverted(true);
    FrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    FrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    BackRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    BackLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //turn variables
    turnTarget = 0;
    turnError = 0;
    turnPrevError = 0;
    turnP = 0;
    turnI = 0;
    turnD = 0;
    turnOutput = 0;
    //drive variables
    driveTarget = 0;
    driveError = 0;
    drivePrevError = 0;
    driveP = 0;
    driveI = 0;
    driveD = 0;
    driveOutput = 0;
  }
//Methods

  //pass in a double input for setting the right side speed.
  public void setRightSpeed(double speed){
    FrontRight.set(ControlMode.PercentOutput, speed);
    BackRight.set(ControlMode.PercentOutput, speed);
  }

  //pass in a double input for setting the left side speed.
  public void setLeftSpeed(double speed){
    FrontLeft.set(ControlMode.PercentOutput, speed);
    BackLeft.set(ControlMode.PercentOutput, speed);
  }

  //takes in sensor input to turn robot into the correct angle
  public void PIDturn(double sensorInput){
    turnPrevError = turnError;
    turnError = turnTarget - sensorInput;
    turnP = turnError;
    turnI += turnError;
    turnD = turnError - turnPrevError;

    turnOutput = Constants.turnkP*turnP + Constants.turnkI*turnI + Constants.turnkD*turnD;
    SmartDashboard.putNumber("PID output:", turnOutput);
    
    // clamp output between -100% and 100%
    // if(output >= 1) output = 1;
    // if(output <= -1) output = -1;

  }

    public void PIDdrive(float sensorInput) {
      
      drivePrevError = driveError;
      driveError = driveTarget - sensorInput;
      driveP = driveError;
      driveI += driveError;
      driveD = driveError - turnPrevError;
      
      driveOutput = Constants.kP*driveP + Constants.kI*driveI + Constants.kD*driveD;
      SmartDashboard.putNumber("PID Drive output:", driveOutput);

      setRightSpeed(driveOutput);
      setLeftSpeed(driveOutput);

    }  

  public void resetEncoders(){
    FrontRight.setSelectedSensorPosition(0);
    FrontLeft.setSelectedSensorPosition(0);
    BackRight.setSelectedSensorPosition(0);
    BackLeft.setSelectedSensorPosition(0);

  }
 
//Getters
  public double getTurnTarget(){
    return turnTarget;
  }

  public double getPIDTurnError(){
    return turnError;
  }

  public double getDriveTarget(){
    return driveTarget;
  }

  public double getDriveError(){
    return driveError;
  }
    
  public double getTurnOutput(){
    return turnOutput;
  }

  public double getDriveOutput(){
    return driveOutput;
  }
//Setters
  public void setTurnTarget(double angle){
    turnTarget = angle;
  }

  public void setDriveTarget(double encoderUnit){
    driveTarget = encoderUnit;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
