// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController Xbox = new XboxController(Constants.XBOX_pin);
  public final AHRS navX = new AHRS(SPI.Port.kMXP);
  
  //subsystems
  private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  public final VisionSubsystems limeLight = new VisionSubsystems();
  public final PixyCamSubsystem pixy2 = new PixyCamSubsystem();
  private final Indexer index = new Indexer();
  public final TurretSubsystem turret = new TurretSubsystem();
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public final Intake intake = new Intake();
  public final JoystickButton buttonA = new JoystickButton(Xbox, Constants.Xbox_Button_A);
  public final JoystickButton buttonB = new JoystickButton(Xbox, Constants.Xbox_Button_B);
  public final JoystickButton buttonX = new JoystickButton(Xbox, Constants.Xbox_Button_X);
  public final JoystickButton buttonY = new JoystickButton(Xbox, Constants.Xbox_Button_Y);
  private final JoystickButton buttonLB = new JoystickButton(Xbox, Constants.Xbox_Button_LB);
  private final JoystickButton buttonRB = new JoystickButton(Xbox, Constants.Xbox_Button_RB);
  private final JoystickButton buttonLS = new JoystickButton(Xbox, Constants.Xbox_Button_LS);
  private final JoystickButton buttonRS = new JoystickButton(Xbox, Constants.Xbox_Button_RS);
  public static final BallCounter ballCounter = new BallCounter();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  
    // Configure the button bindings\
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //initializes the driveTrain for command input, there are a few suppliers
    driveTrain.setDefaultCommand(new JoyStickDrive(driveTrain, () -> Xbox.getRawAxis(Constants.XBOX_L_YAXIS), () -> Xbox.getRawAxis(Constants.XBOX_R_XAXIS)));
    //turret.setDefaultCommand(new TriggerAim(turret, ()-> Xbox.getRightTriggerAxis(), ()-> Xbox.getLeftTriggerAxis()));
    turret.setDefaultCommand(new AutoAim(limeLight, turret));

    // Configure the button bindings
    configureButtonBindings();
  }

  public void robotInit() {

  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    buttonRB.whenHeld(new RunIntake(intake));
    buttonB.whenHeld(new IndexHold(index));
    buttonA.whenHeld(new IndexShoot(index));
    buttonLB.toggleWhenPressed(new ParallelCommandGroup(
      new RunFlywheel(turret, limeLight), new SequentialCommandGroup(
        new IndexReverse(index), new Wait(2000), new IndexShoot(index)
      ))
      );
    buttonX.whenHeld(new IndexReverse(index));
    buttonY.toggleWhenPressed(new TriggerAim(turret, ()-> Xbox.getRightTriggerAxis(), ()-> Xbox.getLeftTriggerAxis()));
    //buttonY.toggleWhenPressed(new AutoAim(limeLight, turret));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    //All commands that should be run in autonomous goes here
    return new SequentialCommandGroup( //parallel command is also possible new parallel command group
      //new AutoTurn(driveTrain, 45.0, () -> navX.getRoll())
      new AutoDrive(driveTrain, 1.0, () -> navX.getDisplacementZ())
      //new AutoTrackingRedBall(pixy2)
      //  new AutoTrackingRedBall(driveTrain, pixy2, () -> navX.getRoll())
            );
  }
}
