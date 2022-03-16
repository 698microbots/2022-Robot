package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallCounter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class AutoIntake extends CommandBase{
    private final Indexer index;
    private final Intake intake;
    int ballnum;
    
    public AutoIntake(Indexer index, Intake intake){
        this.index = index;
        this.intake = intake;
        addRequirements(intake);
        addRequirements(index);
        
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballnum=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballnum = RobotContainer.ballCount;
    if(ballnum==0){
        intake.intputBall();
        index.runLowerIndexer(Constants.indexMotorSpeedBottom);
    }

    if(ballnum==1){
        intake.intputBall();
        index.runLowerIndexer(Constants.indexMotorSpeedBottom);
        index.runUpperIndexer(Constants.indexMotorSpeedTop*-1);
    }

    if(ballnum==2){
        intake.stopMotor();
        index.stopIndexer();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return true;
    }
}
