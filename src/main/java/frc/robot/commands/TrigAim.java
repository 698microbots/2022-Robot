// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TrigAim extends CommandBase {
  /** Creates a new TrigAim. */
  private final TurretSubsystem turret;
  private final Supplier <Double> RT, LT;
  public TrigAim(TurretSubsystem turret, Supplier <Double> RT, Supplier <Double> LT) {
    this.turret = turret;
    this.RT = RT;
    this.LT = LT;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double inputRT = RT.get();
    double inputLT = LT.get();
    turret.turnTurret((inputRT - inputLT)/3);
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
