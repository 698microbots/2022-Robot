// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import java.util.ArrayList;

public class AutoTrackingRedBall extends SubsystemBase {
  /** Creates a new PixyCam. */
  private final Pixy2 pixy2;
  private ArrayList<Block> blocks;
  private int blockCount;

  //Pixy horizontal PID vars
  private double hTarget;
  private double hError;
  private double hPrevError;
  private double hP;
  private double hI;
  private double hD;
  private double hOutput;

  public AutoTrackingRedBall() {
    pixy2 = Pixy2.createInstance(new SPILink());//initialize pixy2 for SPI usage
    pixy2.init();//actually initialize
    pixy2.getCCC().getBlocks(true, 3, 5);//starts the initial calculation done
    blocks = pixy2.getCCC().getBlockCache();//stores all block data into blocks arrayList
    //horizontal PID
    hTarget = 0;//maybe unnecesary
    hError = 0;
    hPrevError = 0;
    hP = 0;
    hI = 0;
    hD = 0;
    hOutput = 0;
  }

  //methods
  public double pixyHorizontalPID(double error){
    hError = error;
    hP = error;
    hI += error;
    hD = error - hPrevError;

    hPrevError = hError;

    hOutput = Constants.pixyHkD*hP + Constants.pixyHkI*hI+Constants.pixyHkD*hD;

    return hOutput;
  }

  //getters
  public int getBlockCount(){//returns the number of blocks there are in the camera's view
    return blockCount;
  }

  public double getBlockXangle(int blockNum){//returns the x angle of specific block from center.
    if(blockCount > 0){
      int x = getBlockXcoordinates(blockNum);
    return ((x - Constants.pixyHcenter)*Constants.pixyHorizontalAngle)/(Constants.pixyWidth/2.0);
    }else{
      return 404.0;
    }
  }

  public double getBlockYangle(int blockNum){//returns the y angle of specific block from center.
    if(blockCount > 0){
    int y = getBlockYcoordinates(blockNum);
    return ((y - Constants.pixyHcenter)*Constants.pixyVerticalAngle)/(Constants.pixyHeight/2.0);
    }else{
      return 404.0;
    }
  }

  public int getBlockSignature(int blockNum){
    if(blockCount > 0){
      return blocks.get(blockNum).getSignature();
    }else{
      return 404;
    }
  }
  public int getBlockXcoordinates(int blockNum){//gives the x cor of specific block
    if(blockCount > 0){
    return blocks.get(blockNum).getX();
    }else{
      return 404;
    }
  }

  public int getBlockYcoordinates(int blockNum){//gives the y cor of specific block
    if(blockCount > 0){
    return blocks.get(blockNum).getY();
    }else{
      return 404;
    }
  }
  
  public int getBallWidth(int blockNum){
    return blocks.get(blockNum).getWidth();
  }

  public int getBallHeight(int blockNum){
    return blocks.get(blockNum).getHeight();
  }

  public double getHWratio(int blockNum){
    return getBallHeight(blockNum)/getBallWidth(blockNum);
  }

  public double getHerror(){
    return hError;
  }
  
  //setters
  public void set_LED_On(){
    pixy2.setLED(255, 255, 255);
  }

  public void set_LED_Off(){
    pixy2.setLED(0, 0, 0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pixy2.getCCC().getBlocks(true, 3, 5);
    blocks = pixy2.getCCC().getBlockCache();
    blockCount = blocks.size();
  }
}