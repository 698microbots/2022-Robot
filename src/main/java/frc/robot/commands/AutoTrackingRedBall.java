// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PixyCam;
import io.github.pseudoresonance.pixy2api.Pixy2;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class AutoTrackingRedBall extends CommandBase {
  /** Creates a new AutoTrackingRedBall. */
  private PixyCam pixy2;
  public AutoTrackingRedBall(PixyCam pixy2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pixy2 = pixy2;
    addRequirements(pixy2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
