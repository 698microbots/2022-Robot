// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.JoyStickDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class turnOffNormalDrive extends CommandBase {
  /** Creates a new turnOffNormalDrice. */
  private final CommandScheduler commandScheduler;
  private final JoyStickDrive joyStickDrive;
   public turnOffNormalDrive(CommandScheduler commandScheduler, JoyStickDrive joyStickDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.commandScheduler = commandScheduler;
    this.joyStickDrive = joyStickDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commandScheduler.cancel(joyStickDrive);
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
