/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.TrajectoryElement;

import java.io.File;
import java.io.FileWriter;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;


import com.kauailabs.navx.frc.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;
  //robot container object used for accessing robot controls
  public static RobotContainer oi;
  public static Drive drive;
  AutoDrive auton;
  public static AHRS navx;
  public static StraightTrajectory trajectoryGenerator;
  int counter = 0;
  
  Timer timer;
  Vision camera;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboadrd.
    // m_robotContainer = new RobotContainer();
    oi = new RobotContainer();
    timer = new Timer();
    drive = new Drive();
    auton = new AutoDrive();
    SmartDashboard.putNumber("number", 5);
    navx = new AHRS(SPI.Port.kMXP);
    trajectoryGenerator = new StraightTrajectory(0.558, 10, 10, 30);
  
    camera = new Vision();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("angle", navx.getAngle());
    drive.resetEncoders();
    SmartDashboard.putNumber("Vertical Angle", camera.getV_angle());
    SmartDashboard.putNumber("Horizontal Angle", camera.getH_angle());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    // auton.initialize();
    // Drive.PIDturnSetTarget(15);
    timer.stop();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // if(Math.abs(Drive.PIDturnGetError()) < 0.1) counter++;
    // else counter=0;
    // if(counter<5){
    //   Drive.PIDturn(navx.getAngle());
    // }
  }


  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
    drive.setDefaultCommand(new JoyStickDrive());
    timer.reset();
    timer.start();
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // drive.leftSpeed(1);
    // drive.rightSpeed(1);
    System.out.println(Double.toString(timer.get()) + " " + Float.toString(navx.getDisplacementX()) + " " + Float.toString(navx.getVelocityX()) + " " + Float.toString(navx.getWorldLinearAccelX()) + "\n");

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    ArrayList<TrajectoryElement> test = new ArrayList<TrajectoryElement>();
    test = trajectoryGenerator.calculateTurn(0, 90, 0, 0, 0.010);
    test.forEach((n) -> System.out.println(Double.toString(n.t) + " " + Double.toString(n.x) + " " + Double.toString(n.v) + " " + Double.toString(n.a) + " " + Double.toString(n.j) + "\n"));
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
