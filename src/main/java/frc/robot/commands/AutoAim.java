// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystems;

public class AutoAim extends CommandBase {
  /** Creates a new AutoAim. */
  private final VisionSubsystems limelight;
  private final TurretSubsystem turret;
  public AutoAim(VisionSubsystems limelight, TurretSubsystem turret) {
    this.limelight = limelight;
    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.tracking()){
      turret.turnTurret(turret.turretPID(limelight.getH_angle()));
    }else{
      turret.turnTurret(turret.turretPID(-turret.getTurretAngle()));//sets the turret back to center if not tracking
    }
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
