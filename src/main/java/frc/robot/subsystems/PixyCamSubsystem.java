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
  private Block block0;
  private Block block1;
  private Block block2;
  private Block block3;
  private Block block4;

  public PixyCamSubsystem() {
    pixy2 = Pixy2.createInstance(new SPILink());//initialize pixy2 for SPI usage
    pixy2.init();//actually initialize
    pixy2.getCCC().getBlocks(true, 3, 5);//starts the initial calculation done
    blocks = pixy2.getCCC().getBlockCache();//stores all block data into blocks arrayList
  }

  //Block0 info
  public int getBlock0Xcoordinates(){//returns the Xcoordinates of block
    return block0.getX();//obtain the x-cor from the biggest block object.
  }

  public int getBlock0Ycoordinates(){//returns the Ycoordinates of block
    return block0.getY();
  }

  public int getBlock0Signature(){//return the signature of block
    return block0.getSignature();
  }

  public double getBlock0Angle(){//return angle from the camera???
    return block0.getAngle();
  }

  //Block1 Info
  public int getBlock1Xcoordinates(){
    return block1.getX();
  }

  public int getBlock1Ycoordinates(){
    return block1.getY();
  }

  public int getBlock1Signature(){
    return block1.getSignature();
  }  
  
  public double getBlock1Angle(){
    return block1.getAngle();
  }
  
  //Block2 Info
  public int getBlock2Xcoordinates(){
    return block2.getX();
  }

  public int getBlock2Ycoordinates(){
    return block2.getY();
  }

  public int getBlock2Signature(){
    return block2.getSignature();
  }  
  
  public double getBlock2Angle(){
    return block2.getAngle();
  }
  
  //Block3 Info
  public int getBlock3Xcoordinates(){
    return block3.getX();
  }

  public int getBlock3Ycoordinates(){
    return block3.getY();
  }
  
  public int getBlock3Signature(){
    return block3.getSignature();
  }  
  
  public double getBlock3Angle(){
    return block3.getAngle();
  }
  
  //Block4 Info
  public int getBlock4Xcoordinates(){
    return block4.getX();
  }

  public int getBlock4Ycoordinates(){
    return block4.getY();
  }

  public int getBlock4Signature(){
    return block4.getSignature();
  }  
  
  public double getBlock4Angle(){
    return block4.getAngle();
  }
  
  //lens can see 60 degrees horizontal, 40 degrees vertical (30 degrees horizontally and 20 degrees vertically to each side)
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pixy2.getCCC().getBlocks(true, 3, 5);
    blocks = pixy2.getCCC().getBlockCache();
    if(blocks.size() > 0){
      block0 = blocks.get(0);//create an object of the biggest block.
      block1 = blocks.get(1);
      block2 = blocks.get(2);
      block3 = blocks.get(3);
      block4 = blocks.get(4);  
    }
  }
}
