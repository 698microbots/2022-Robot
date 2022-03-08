// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import java.util.ArrayList;
import java.util.function.Supplier;

public class PixyCamSubsystem extends SubsystemBase {
  /** Creates a new PixyCam. */
  private final Pixy2 pixy2;
  private ArrayList<Block> blocks;
  private int blockCount;

  public PixyCamSubsystem() {
    pixy2 = Pixy2.createInstance(new SPILink());//initialize pixy2 for SPI usage
    pixy2.init();//actually initialize
    pixy2.getCCC().getBlocks(true, 3, 5);//starts the initial calculation done
    blocks = pixy2.getCCC().getBlockCache();//stores all block data into blocks arrayList
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
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pixy2.getCCC().getBlocks(true, 3, 5);
    blocks = pixy2.getCCC().getBlockCache();
    blockCount = blocks.size();
  }

public static int getBlockXCoordinate() {
    return 0;
}

public static int getBlockYCoordinate() {
    return 0;
}
}
