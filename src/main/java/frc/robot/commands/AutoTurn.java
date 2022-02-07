// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoTurn extends CommandBase {
  /** Creates a new AutoTurn. */
  private final DriveTrain driveTrain;
  private final double target;
  private final Supplier<Double> navXInput;
  private int counter;

  public AutoTurn(DriveTrain driveTrain, double target, Supplier<Double> navXInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.target = target;
    counter = 0;
    this.navXInput = navXInput;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Automatic turning has started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.setTarget(target);
    double sensorInput = navXInput.get();
    driveTrain.PIDturn(sensorInput);

    //Increment the counter when error is small enough
    if(driveTrain.getPIDTurnError() < 0.1){
      counter++;
    }else{
      counter = 0;
    }

    //reset encoders
    driveTrain.resetEncoders();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Automatic turning has ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if the counter is big enough, this method returns true causing the command to end.
    if(counter > 10){
      return true;
    }else{
      counter = 0;
    return false;
    }
  }
}
