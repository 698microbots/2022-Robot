// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallCounterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoIntake extends CommandBase {
  /** Creates a new AutoIntake. */
  private final BallCounterSubsystem ballCounter;
  private final IntakeSubsytem intake;
  private final IndexerSubsystem index;
  public AutoIntake(BallCounterSubsystem ballCounter, IntakeSubsytem intake, IndexerSubsystem index) {
    this.ballCounter = ballCounter;
    this.intake = intake;
    this.index = index;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballCounter);
    addRequirements(intake);
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intputBall();
    index.runLowerIndexer(Constants.indexMotorSpeedBottom);
    RobotContainer.Xbox.setRumble(RumbleType.kRightRumble, 1.0); // ?? any real reason for rumble in controller
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    index.stopIndexer();
    RobotContainer.Xbox.setRumble(RumbleType.kRightRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ballCounter.bottomSensorStatus() == true){
      return true; //?? explain difference between end and isFinished again
    }else{ // when this is true, end is run
      return false;
    }
  }
}
