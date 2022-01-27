/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.*;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends SubsystemBase {
  /**
   * Creates a new Drive.
   */
  private static TalonFX FrontRight = new TalonFX(Constants.FrontRightID);
  private static TalonFX FrontLeft = new TalonFX(Constants.FrontLeftID);
  private static TalonFX BackRight = new TalonFX(Constants.BackRightID);
  private static TalonFX BackLeft = new TalonFX(Constants.BackLeftID);  
 

  public Drive() {
  //setting both right motors to be inverted
    FrontLeft.setInverted(true);
    BackLeft.setInverted(true);
    //using sensor built in to the motor
    FrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    FrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    BackRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    BackLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


  }
//sets the speed for both left motors
  public void leftSpeed(double speed)
  {
    FrontLeft.set(ControlMode.PercentOutput, speed);
    BackLeft.set(ControlMode.PercentOutput, speed);
  }

//sets the speed for both right motors 
  public void rightSpeed(double speed)
  {
    FrontRight.set(ControlMode.PercentOutput, speed);
    BackRight.set(ControlMode.PercentOutput, speed);
  }

//takes distance in encoder units and drives with PID
  public void PIDdrive(double distance)
  {
    //SmartDashboard.putNumber(FrontRight.getSensorCollection().getQuadraturePosition());
    //FrontRight
    FrontRight.configNominalOutputForward(0,Constants.kTimeoutMs);    
    FrontRight.configNominalOutputReverse(0,Constants.kTimeoutMs);
    FrontRight.configPeakOutputForward(0.5,Constants.kTimeoutMs);    
    FrontRight.configPeakOutputReverse(-0.5,Constants.kTimeoutMs);

    FrontRight.configForwardSoftLimitThreshold(10000,0);
    FrontRight.configReverseSoftLimitThreshold(-10000,0);
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
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
