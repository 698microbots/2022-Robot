// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSubsytem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final XboxController Xbox = new XboxController(Constants.XBOX_pin);
  public final AHRS navX = new AHRS(SerialPort.Port.kUSB);
  
  //subsystems
  public DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  public final VisionSubsystems limeLight = new VisionSubsystems();
  private final VisionSubsystems limeLight2 = new VisionSubsystems();
  private final IndexerSubsystem index = new IndexerSubsystem();
  private final FlyWheelSubsystem flyWheel = new FlyWheelSubsystem();
  // private final IndexerSubsystem index
  public final TurretSubsystem turret = new TurretSubsystem();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public final IntakeSubsytem intake = new IntakeSubsytem();
  public final JoystickButton buttonA = new JoystickButton(Xbox, Constants.Xbox_Button_A);
  public final JoystickButton buttonB = new JoystickButton(Xbox, Constants.Xbox_Button_B);
  public final JoystickButton buttonX = new JoystickButton(Xbox, Constants.Xbox_Button_X);
  public final JoystickButton buttonY = new JoystickButton(Xbox, Constants.Xbox_Button_Y);
  private final JoystickButton buttonLB = new JoystickButton(Xbox, Constants.Xbox_Button_LB);
  private final JoystickButton buttonRB = new JoystickButton(Xbox, Constants.Xbox_Button_RB);
  private final JoystickButton buttonLS = new JoystickButton(Xbox, Constants.Xbox_Button_LS);
  private final JoystickButton buttonRS = new JoystickButton(Xbox, Constants.Xbox_Button_RS);
  public static BallCounterSubsystem ballCounter = new BallCounterSubsystem();
  public static int ballCount = 0;


  /**d
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  
    // Configure the button bindings
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //initializes the driveTrain for command input, there are a few suppliers
    driveTrain.setDefaultCommand(new JoyStickDrive(driveTrain, () -> Xbox.getRawAxis(Constants.XBOX_L_YAXIS), () -> Xbox.getRawAxis(Constants.XBOX_R_XAXIS)));
    turret.setDefaultCommand(new TriggerAim(turret, ()-> Xbox.getRawAxis(Constants.Xbox_RT), ()-> Xbox.getRawAxis(Constants.Xbox_LT)));
    //turret.setDefaultCommand(new AutoAim(limeLight, turret));
    // ballCounter.setDefaultCommand(new CountBalls(ballCounter));
    

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

    buttonRB.toggleWhenPressed(new ParallelCommandGroup(new IndexHold(index), new RunIntake(intake)));
    buttonB.whenHeld(new IndexHold(index));
    buttonA.whenHeld(new IndexShoot(index, ballCounter));
    buttonX.whenHeld(new ParallelCommandGroup(new IntakeReverse(intake), new TeleopRejected(index)));
    buttonY.toggleWhenPressed(new TeleopAutoAim(limeLight, turret));
    buttonRS.whenPressed(new RecenterTurret(turret));

    //Command Groups
    buttonLB.toggleWhenPressed(new SequentialCommandGroup(
      //shoot first ball
      new ParallelCommandGroup(
      new RunFlywheel(flyWheel, limeLight), new SequentialCommandGroup(
        new AutonAim(limeLight2, turret),//if limeLight not in range, will shoot randomly
        new IndexReverse(index),
        new Wait(500),
        new IndexShoot(index, ballCounter)
      )),
      //shoots second ball
      new ParallelCommandGroup(
        new RunFlywheel(flyWheel, limeLight), new SequentialCommandGroup(
          new AutonAim(limeLight2, turret),//if limeLight not in range, will shoot randomly
          new IndexReverse(index),
          new Wait(500),
          new IndexShoot(index, ballCounter)
        )),
        new RecenterTurret(turret)//might mess up stabilization
      ));
   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    //All commands that should be run in autonomous goes here

    //sequence 1
    // return new SequentialCommandGroup( //parallel command is also possible new parallel command group
    //   new SpinFlyWheelAt(flyWheel, 0.5),//20ms
    //   new AutoDrive(driveTrain, -85.0, 3500, 0.7),//1500ms
    //    new AutonAim(limeLight, turret),  //2000ms
    //   new ParallelCommandGroup(new RunFlywheel(flyWheel, limeLight),//300ms
    //    new SequentialCommandGroup(
    //     new IndexReverse(index),//80ms
    //     new Wait(Constants.HoldTime),//500ms
    //     new IndexShoot(index, ballCounter))),//200ms
    //    new AutoTurn(driveTrain, 52.5, navX, 2500),//2000ms
    //     new ParallelCommandGroup(new AutoDrive(driveTrain, 60, 1500, 0.5), new AutoIntake(ballCounter, intake, index)),//1500ms
    //     //  new TurnTurretTo(turret, -37.5),
    //     new AutoIndexHold(index, ballCounter),
    //     new AutoTurn(driveTrain, -30, navX, 3000),//1500ms
    //      new AutonAim(limeLight, turret),//2000ms
    //      new ParallelCommandGroup(new RunFlywheel(flyWheel, limeLight), new SequentialCommandGroup(//300ms
    //      new IndexReverse(index),//20ms
    //      new Wait(Constants.HoldTime),//500ms
    //      new IndexShoot(index, ballCounter))),//200ms

    //      //drive help
    //      new AutoDrive(driveTrain, -40, 1000, 1),//1000ms
    //      new AutoTurn(driveTrain, 180, navX, 2000),//1000ms
    //     new RecenterTurret(turret)//rest of the time
    // );

    //sequence 2
    return new SequentialCommandGroup(
      new SpinFlyWheelAt(flyWheel, 0.45),
      new AutoDrive(driveTrain, -30, 1500, 1),
      new ParallelCommandGroup(new AutoDrive(driveTrain, 90, 2000, 0.3), new AutoIntake(ballCounter, intake, index)),
      new AutoTurn(driveTrain, 180, navX, 3500),
      new AutoIndexHold(index, ballCounter),
      new SequentialCommandGroup(
      //shoot first ball
      new ParallelCommandGroup(
      new RunFlywheel(flyWheel, limeLight), new SequentialCommandGroup(
        new AutonAim(limeLight2, turret),//if limeLight not in range, will shoot randomly
        //new IndexReverse(index),
        new Wait(1000),
        new IndexShoot(index, ballCounter)
      )),
      //shoots second ball
      new ParallelCommandGroup(
        new RunFlywheel(flyWheel, limeLight), new SequentialCommandGroup(
          new AutonAim(limeLight2, turret),//if limeLight not in range, will shoot randomly
          //new IndexReverse(index),
          new Wait(1000),
          new IndexShoot(index, ballCounter)
        )),
        new RecenterTurret(turret)//might mess up stabilization
      )
       );
  }

  public Command getTestCommand(){
    //return new AutoDrive(driveTrain, -90, 2500, 1);
    return new AutoTurn(driveTrain, -90, navX, 3000);
  }
}
