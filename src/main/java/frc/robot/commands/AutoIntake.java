// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallCounterSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.RobotContainer;

public class AutoIntake extends CommandBase {
  /** Creates a new AutoIntake. */
  private final BallCounterSubsystem ballCounter;
  private final IntakeSubsytem intake;
  public AutoIntake(BallCounterSubsystem ballCounter, IntakeSubsytem intake) {
    this.ballCounter = ballCounter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballCounter);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intputBall();
    RobotContainer.Xbox.setRumble(RumbleType.kRightRumble, 1.0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    RobotContainer.Xbox.setRumble(RumbleType.kRightRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ballCounter.bottomSensorStatus() == true){
      return true;
    }else{
      return false;
    }
  }
}
