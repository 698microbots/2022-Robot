// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class RecenterTurret extends CommandBase {
  /** Creates a new RecenterTurret. */
  private final TurretSubsystem turret;
  private int counter;
  public RecenterTurret(TurretSubsystem turret) {
    this.turret = turret;
    counter = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.turnTurret(turret.getTurretAngle());
    if(turret.getTurretAngle()<= 1){
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
    if(counter >= 2000/20){
      return true;
    }else{
      return false;
    }
  }
}
