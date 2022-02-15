// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PixyCamSubsystem;
import io.github.pseudoresonance.pixy2api.Pixy2;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class AutoTrackingRedBall extends CommandBase {
  /** Creates a new AutoTrackingRedBall. */
  private PixyCamSubsystem pixy2;
  private DriveTrainSubsystem driveTrainSubsystem;
  private Supplier <Double> navXinput;
  public AutoTrackingRedBall(PixyCamSubsystem pixy2, DriveTrainSubsystem driveTrainSubsystem, Supplier<Double> navXinput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pixy2 = pixy2;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.navXinput = navXinput;
    addRequirements(pixy2);
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrainSubsystem.setTurnTarget(driveTrainSubsystem.getTurnTarget());
    double sensorInput = navXinput.get();
    driveTrainSubsystem.PIDturn(sensorInput);
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
