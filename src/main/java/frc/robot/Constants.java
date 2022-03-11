/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
//Motor Constants
    public static final int FrontRightID = 0;
    public static final int FrontLeftID = 3;
    public static final int BackRightID = 5;
    public static final int BackLeftID = 2;

//Controller Stick Constants
    public static final int XBOX_R_XAXIS = 4;
    public static final int XBOX_R_YAXIS = 5;
    public static final int XBOX_L_XAXIS = 0;
    public static final int XBOX_L_YAXIS = 1;
    public static final int XBOX_pin = 0;
    public static final int Xbox_LT = 2;
    public static final int Xbox_RT = 3;


    //Controller Button IDs
    public static final int Xbox_Button_A = 1;
    public static final int Xbox_Button_B = 2;
    public static final int Xbox_Button_X = 3;
    public static final int Xbox_Button_Y = 4;
    public static final int Xbox_Button_LB = 5;
    public static final int Xbox_Button_RB = 6;
    public static final int Xbox_Button_LS = 9;
    public static final int Xbox_Button_RS = 10;

    // auton drive PID constants
    public static final int kTimeoutMs = 20;
    public static final int kPIDLoopIdx = 0;//run primary loop
    public static final double kF = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0.5;

    // Index Constants
    public static final double indexMotorSpeedBottom = 0.5;
    public static final double indexMotorSpeedTop = 0.5;


    // Intake Constants
    public static final int deviceIdIntake = 6;
    public static final double intakeMotorSpeed = 0.42;
    public static final int pistonID = 1;   
    public static final double ampSpike = .1;
    public static final double closestY = 100;
    public static final double closestX = 160;
    // turn PID constants
    public static final double turnkP = 0.01;
    public static final double turnkI = 0;
    public static final double turnkD = 0;

    // Vision constants
    public static final double goalHeight = 107;

    //PixyCamera Constants
    public static final int pixyWidth = 316;
    public static final int pixyHeight = 208;
    public static final int pixyHcenter = 158;
    public static final int pixyVerticalAngle = 20;
    public static final int pixyHorizontalAngle = 30;
    public static final int pixyRedSig = 1;
    public static final int pixyBlueSig = 2;

    //AutoIntake
    public static final int x1Range = 0;
    public static final int y1Range = 20;
    public static final int x2Range = 315;
    public static final int y2Range = 207;

    // Photovoltaic Sensor IDs
    public static final int PhotovoltaicID1 = 8;
    public static final int PhotovoltaicID2 = 3;

    //Auto ball driving
    public static final int turnAggressiveness = 2;

    //Turret Constants
    public static final int turrentMotorID = 6;//need to figure out what this value is
    public static final double turretkP = 0.01;
    public static final double turretkI = 0;
    public static final double turretkD = 0;
    public static final int flyWheelMotorID = 1;
    public static final double turretMotorGearRatio = 98;
    public static final double flyWheelSpeed = 0.8;
    public static final double turretMaxAngle = 62;
    public static final double turretMinAngle = -62; 

    //SparkMax Motor IDs
    public static final int lowerIndexerID = 9;
    public static final int upperIndexerID = 10;

    // Ball counter constants
    public static final int PortID1 = 8;
    public static final int PortID2 = 3;

}
