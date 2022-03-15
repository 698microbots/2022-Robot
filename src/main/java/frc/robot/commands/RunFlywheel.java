// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystems;

public class RunFlywheel extends CommandBase {
  /** Creates a new TestFlywheel. */
  private final TurretSubsystem turret;
  private final VisionSubsystems limelight;
  private double flyWheelSpeed;

  public RunFlywheel(TurretSubsystem turret, VisionSubsystems limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;
    flyWheelSpeed = 0.0;
    addRequirements(turret);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //calculate flywheel speed
    flyWheelSpeed = limelight.getZdistance()*0.00127 + 0.439;
    
    //clamp down maximum speed
    if(flyWheelSpeed > 0.68){
      flyWheelSpeed = 0.68;
    }

    //put the calculated value in to runFlywheel
    turret.runFlywheel(flyWheelSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
