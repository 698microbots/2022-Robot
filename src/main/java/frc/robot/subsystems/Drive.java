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

import java.util.function.LongToDoubleFunction;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.*;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

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

//auton: takes distance in encoder units and drives with PID
  public void PIDdrive(double distance)
  {
    // set PID sensor source
    FrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    FrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    BackRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    BackLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

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

    FrontRight.config_kF(Constants.kPIDLoopIdx, Constants.drivekF, Constants.kTimeoutMs);
    FrontRight.config_kD(0, Constants.drivekD, Constants.kTimeoutMs);
    FrontRight.config_kI(0, Constants.drivekI, Constants.kTimeoutMs);
    FrontRight.config_kP(0, Constants.drivekP, Constants.kTimeoutMs);

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

    FrontLeft.config_kF(Constants.kPIDLoopIdx, Constants.drivekF, Constants.kTimeoutMs);
    FrontLeft.config_kD(0, Constants.drivekD, Constants.kTimeoutMs);
    FrontLeft.config_kI(0, Constants.drivekI, Constants.kTimeoutMs);
    FrontLeft.config_kP(0, Constants.drivekP, Constants.kTimeoutMs);

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

    BackRight.config_kF(Constants.kPIDLoopIdx, Constants.drivekF, Constants.kTimeoutMs);
    BackRight.config_kD(0, Constants.drivekD, Constants.kTimeoutMs);
    BackRight.config_kI(0, Constants.drivekI, Constants.kTimeoutMs);
    BackRight.config_kP(0, Constants.drivekP, Constants.kTimeoutMs);

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

    BackLeft.config_kF(Constants.kPIDLoopIdx, Constants.drivekF, Constants.kTimeoutMs);
    BackLeft.config_kD(0, Constants.drivekD, Constants.kTimeoutMs);
    BackLeft.config_kI(0, Constants.drivekI, Constants.kTimeoutMs);
    BackLeft.config_kP(0, Constants.drivekP, Constants.kTimeoutMs);

    FrontRight.set(ControlMode.Position, distance);
    FrontLeft.set(ControlMode.Position, distance);
    BackRight.set(ControlMode.Position, distance);
    BackLeft.set(ControlMode.Position, distance);
  }

  // // PID from scratch; needs to be tuned
  // public void turnPID(double angle) {

  //   double error = angle-Robot.navx.getAngle();
  //   double proportional = 0;
  //   double integral = 0;
  //   double derivative = 0;
  //   double output = 0;
  //   double previousError = 0;

  //   Long timeout = (long) 20;

  //   while (true){
  //     error = angle-Robot.navx.getAngle();
  //     proportional = error;
  //     integral += error; // times timeout?
  //     derivative = error-previousError; //times timeout?
  //     output = Constants.turnKp * proportional + Constants.turnKi * integral + Constants.turnKd * derivative;

  //     rightSpeed(-output);
  //     leftSpeed(output);

  //     try {
  //         Thread.sleep(timeout);
  //     } catch (InterruptedException e) {
  //       // TODO Auto-generated catch block
  //       e.printStackTrace();
  //     }
        
  //     previousError = error;
  //   }
  // }

  // Reset encoders
  public void resetEncoders(){
    FrontLeft.setSelectedSensorPosition(0);
    FrontRight.setSelectedSensorPosition(0);
    BackLeft.setSelectedSensorPosition(0);
    BackRight.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
