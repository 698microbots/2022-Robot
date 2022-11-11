// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hangerPneumatics;


public class hangerControls extends CommandBase {
  /** Creates a new hangerControls. */
  private final Supplier<Boolean> rightBumper, leftBumper, AButton, BButton;
  private final hangerPneumatics hangerPneumatics;

  public hangerControls(hangerPneumatics hangerPneumatics, Supplier<Boolean> rightBumper, Supplier<Boolean> leftBumper, Supplier<Boolean> AButton, Supplier<Boolean> BButton) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rightBumper = rightBumper;
    this.leftBumper = leftBumper;
    this.AButton = AButton;
    this.BButton = BButton;
    this.hangerPneumatics = hangerPneumatics;
    addRequirements(hangerPneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rightBumper.get() == true){
      hangerPneumatics.setSolForward();
    } else if (leftBumper.get() == true){
      hangerPneumatics.setSolBackward();
    }

    if (AButton.get() == true){
      hangerPneumatics.turnComOn();
    } else if (BButton.get() == true){
      hangerPneumatics.turnComOff();
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
