// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//2048 encoder position is ONE ROTATION
package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import java.math.*;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;


public class positioningSystem extends CommandBase {
  /** Creates a new positioningSystem. */
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final AHRS navx;
  private double XRoboPosition = 0;
  private double YRoboPosition = 0;
  // private double navxAngle = 0;
  // private double rightDist = 0;
  // private double leftDist = 0;
  // private double rotationRightCount = 0;
  // private double rotationLeftCount = 0;
  public positioningSystem(DriveTrainSubsystem driveTrainSubsystem, AHRS navx) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.navx = navx;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute() {
  // Pose2d robotPos = odometry.getPoseMeters();
    // if (driveTrainSubsystem.getRightEncoders() % 2048 == 0){
    //   rotationRightCount++;
    // } else if (driveTrainSubsystem.getLeftEncoders() % 2048 == 0){
    //   rotationLeftCount++;
    // }
    
    // navxAngle = navx.getAngle();
    // rightDist = (driveTrainSubsystem.getRightEncoders() / 2048) * (9.652 * Math.PI); // takes the encoders as %, multiplies by circumference
    // leftDist = (driveTrainSubsystem.getLeftEncoders() / 2048) * (9.652 * Math.PI); // the second part after the * is the simplified circumference

    XRoboPosition = navx.getDisplacementX();
    YRoboPosition = navx.getDisplacementY();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
