// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystems;

public class AutonAim extends CommandBase {
  /** Creates a new AutoAim. */
  private final VisionSubsystems limelight;
  private final TurretSubsystem turret;
  private int counter;

  public AutonAim(VisionSubsystems limelight, TurretSubsystem turret) {
    this.limelight = limelight;
    this.turret = turret;
    counter = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      turret.turnTurret(turret.turretPID(limelight.getH_angle()));
      if(Math.abs(turret.getTurretError()) < 3.0){
        counter++;
      }else{
        counter = 0;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.turnTurret(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter > 3){
      return true;
    }else{
      return false;
    } 
  }
}
