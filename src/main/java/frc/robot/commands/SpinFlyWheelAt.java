// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SpinFlyWheelAt extends CommandBase {
  /** Creates a new SpinFlyWheelAt. */
  private final FlyWheelSubsystem flyWheel;
  private final double speed;
  public SpinFlyWheelAt(FlyWheelSubsystem flyWheel, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flyWheel = flyWheel;
    this.speed = speed;
    addRequirements(flyWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flyWheel.runFlywheel(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flyWheel.runFlywheel(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flyWheel.runFlywheel(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
