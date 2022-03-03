// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

//for teleop, pair with flywheel to the same button

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private static CANSparkMax indexer;
  private static BallCounter countBalls;
  public Indexer() {
    indexer = new CANSparkMax(Constants.CANIndexerID, CANSparkMax.MotorType.kBrushless);
    countBalls = new BallCounter();
  }

  public static void shootBall(int numBalls)
  {
    while(numBalls>0)
    {
      indexer.set(0.5);
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      indexer.set(0.0);
      numBalls = countBalls.getBalls();
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
