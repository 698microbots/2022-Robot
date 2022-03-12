// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Robot;
// import frc.robot.subsystems.DriveTrainSubsystem;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.PixyCamSubsystem;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import io.github.pseudoresonance.pixy2api.*;
// import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
// import io.github.pseudoresonance.pixy2api.links.SPILink;

// public class AutoIntake extends SequentialCommandGroup {
//   /** Creates a new AutoIntake. */
//   private final DriveTrainSubsystem driveTrain;
//   private final Intake intake;
//   private final PixyCamSubsystem pixy;

  
//   public AutoIntake(DriveTrainSubsystem driveTrain, Intake intake, PixyCamSubsystem pixy) {
//     this.driveTrain = driveTrain;
//     this.intake = intake;
//     this.pixy = pixy;
    


//     // want to get max of array using max()
//     // how to use drivetrain angle function
//     // auto turn is the pid for target?


//     addRequirements(driveTrain);
//     addRequirements(intake);
//     addRequirements(pixy);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   // Import the subsystems
//   @Override
//   public void initialize() {
    
//   }


//   // Called every time the scheduler runs while the command is scheduled.
//   // Set parameters for when it should move
//   // Take the largest object in array
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   // Actually move the robot
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
