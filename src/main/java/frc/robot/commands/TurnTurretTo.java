// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurnTurretTo extends CommandBase {
  /** Creates a new RecenterTurret. */
  private final TurretSubsystem turret;
  private final double angle;
  private int counter;
  public TurnTurretTo(TurretSubsystem turret, double angle) {
    this.turret = turret;
    this.angle = angle;
    counter = 0;
    // Use addRequirements() here to declare subsystem dependencies.
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
    turret.turnTurret(-(turret.turretPID(angle - turret.getTurretAngle())/2));
    if(Math.abs(turret.getTurretAngle())<= 1){
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
    if(counter >= 10){
      return true;
    }else{
      return false;
    }
  }
}
