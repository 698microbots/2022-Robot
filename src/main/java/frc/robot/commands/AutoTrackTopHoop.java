// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.TurretSubsystem;
// import frc.robot.subsystems.VisionSubsystems;

// public class AutoTrackTopHoop extends CommandBase {
//   /** Creates a new AutoTrackTopHoop. */
//   private final TurretSubsystem turret;
//   private final VisionSubsystems limelight;

//   public AutoTrackTopHoop(TurretSubsystem turret, VisionSubsystems limelight) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.turret = turret;
//     this.limelight = limelight;
//     addRequirements(turret);
//     addRequirements(limelight);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     turret.turnTurret(limelight.getH_angle());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     turret.turnTurret(0.0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
