// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final TalonFX FrontRight = new TalonFX(Constants.FrontRightID);
  private final TalonFX FrontLeft = new TalonFX(Constants.FrontLeftID);
  private final TalonFX BackRight = new TalonFX(Constants.BackRightID);
  private final TalonFX BackLeft = new TalonFX(Constants.BackLeftID);

  //PIDturn variables
  private double target;
  private double error;
  private double prevError;
  private double P;
  private double I;
  private double D;
  private double output;

//Constructors

  public DriveTrain() {
    //initialize all neccessary variables and sensors.
    FrontRight.setInverted(false);
    FrontLeft.setInverted(true);
    BackRight.setInverted(false);
    BackLeft.setInverted(true);
    FrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    FrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    BackRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    BackLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    target = 0;
    error = 0;
    prevError = 0;
    P = 0;
    I = 0;
    D = 0;
    output = 0;
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
    prevError = error;
    error = target - sensorInput;
    P = error;
    I += error;
    D = error - prevError;

    output = Constants.turnkP*P + Constants.turnkI*I + Constants.turnkD*D;

    // clamp output between -100% and 100%
    if(output >= 1) output = 1;
    if(output <= -1) output = -1;

    // set motors to output: left side positive, right side negative for clockwise rotation
    setRightSpeed(output);
    setLeftSpeed(-output);
  }

  public void PIDdrive(double distance)
  {
    //FrontRight
    FrontRight.configNominalOutputForward(0,Constants.kTimeoutMs);    
    FrontRight.configNominalOutputReverse(0,Constants.kTimeoutMs);
    FrontRight.configPeakOutputForward(0.5,Constants.kTimeoutMs);    
    FrontRight.configPeakOutputReverse(-0.5,Constants.kTimeoutMs);

    FrontRight.configForwardSoftLimitThreshold(10000,0);//this seems awfully suspicious, limit???
    FrontRight.configReverseSoftLimitThreshold(-10000,0);//is this where all our ghost limits are coming from?
    FrontRight.configForwardSoftLimitEnable(true,0);
    FrontRight.configReverseSoftLimitEnable(true,0);

    FrontRight.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    FrontRight.config_kF(Constants.kPIDLoopIdx, Constants.kF, Constants.kTimeoutMs);
    FrontRight.config_kD(0, Constants.kD, Constants.kTimeoutMs);
    FrontRight.config_kI(0, Constants.kI, Constants.kTimeoutMs);
    FrontRight.config_kP(0, Constants.kP, Constants.kTimeoutMs);

    //FrontLeft
    FrontLeft.configNominalOutputForward(0,Constants.kTimeoutMs);    
    FrontLeft.configNominalOutputReverse(0,Constants.kTimeoutMs);
    FrontLeft.configPeakOutputForward(0.5,Constants.kTimeoutMs);    
    FrontLeft.configPeakOutputReverse(-0.5,Constants.kTimeoutMs);

    FrontLeft.configForwardSoftLimitThreshold(10000,0);
    FrontLeft.configReverseSoftLimitThreshold(-10000,0);
    FrontLeft.configForwardSoftLimitEnable(true,0);
    FrontLeft.configReverseSoftLimitEnable(true,0);

    FrontLeft.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    FrontLeft.config_kF(Constants.kPIDLoopIdx, Constants.kF, Constants.kTimeoutMs);
    FrontLeft.config_kD(0, Constants.kD, Constants.kTimeoutMs);
    FrontLeft.config_kI(0, Constants.kI, Constants.kTimeoutMs);
    FrontLeft.config_kP(0, Constants.kP, Constants.kTimeoutMs);

    //BackRight
    BackRight.configNominalOutputForward(0,Constants.kTimeoutMs);    
    BackRight.configNominalOutputReverse(0,Constants.kTimeoutMs);
    BackRight.configPeakOutputForward(0.5,Constants.kTimeoutMs);    
    BackRight.configPeakOutputReverse(-0.5,Constants.kTimeoutMs);

    BackRight.configForwardSoftLimitThreshold(10000,0);
    BackRight.configReverseSoftLimitThreshold(-10000,0);
    BackRight.configForwardSoftLimitEnable(true,0);
    BackRight.configReverseSoftLimitEnable(true,0);

    BackRight.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    BackRight.config_kF(Constants.kPIDLoopIdx, Constants.kF, Constants.kTimeoutMs);
    BackRight.config_kD(0, Constants.kD, Constants.kTimeoutMs);
    BackRight.config_kI(0, Constants.kI, Constants.kTimeoutMs);
    BackRight.config_kP(0, Constants.kP, Constants.kTimeoutMs);

    //BackLeft
    BackLeft.configNominalOutputForward(0,Constants.kTimeoutMs);    
    BackLeft.configNominalOutputReverse(0,Constants.kTimeoutMs);
    BackLeft.configPeakOutputForward(0.5,Constants.kTimeoutMs);    
    BackLeft.configPeakOutputReverse(-0.5,Constants.kTimeoutMs);

    BackLeft.configForwardSoftLimitThreshold(10000,0);
    BackLeft.configReverseSoftLimitThreshold(-10000,0);
    BackLeft.configForwardSoftLimitEnable(true,0);
    BackLeft.configReverseSoftLimitEnable(true,0);

    BackLeft.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    BackLeft.config_kF(Constants.kPIDLoopIdx, Constants.kF, Constants.kTimeoutMs);
    BackLeft.config_kD(0, Constants.kD, Constants.kTimeoutMs);
    BackLeft.config_kI(0, Constants.kI, Constants.kTimeoutMs);
    BackLeft.config_kP(0, Constants.kP, Constants.kTimeoutMs);

    FrontRight.set(ControlMode.Position, distance);
    FrontLeft.set(ControlMode.Position, distance);
    BackRight.set(ControlMode.Position, distance);
    BackLeft.set(ControlMode.Position, distance);

    resetEncoders();
  }

  public void resetEncoders(){
    FrontRight.setSelectedSensorPosition(0);
    FrontLeft.setSelectedSensorPosition(0);
    BackRight.setSelectedSensorPosition(0);
    BackLeft.setSelectedSensorPosition(0);

  }
 
//Getters
  public double getTarget(){
    return target;
  }

  public double getPIDTurnError(){
    return error;
  }

//Setters
  public void setTarget(double angle){
    target = angle;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
