// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import java.util.ArrayList;
import java.util.function.Supplier;

public class PixyCam extends SubsystemBase {
  /** Creates a new PixyCam. */
  private final Pixy2 pixy2;
  private ArrayList<Block> blocks;
  private int blockCount;

  public PixyCam() {
    pixy2 = Pixy2.createInstance(new SPILink());//initialize pixy2 for SPI usage
    pixy2.init();//actually initialize
    blockCount = pixy2.getCCC().getBlocks(false, 1, 255);//wait for next frame with target object, the signature of object, how many object tracking?
    pixy2.getCCC().getBlocks(false, 1, 255);
    blocks = pixy2.getCCC().getBlockCache();
  }

  //getters
  public int getRedXcordinate(){
    blockCount = pixy2.getCCC().getBlocks(false, 3, 255);
    pixy2.getCCC().getBlocks(false, 1, 255);
    blocks = pixy2.getCCC().getBlockCache();
    try {
      return blocks.get(0).getX();
    } catch (Exception ArrayIndexOutOfBoundsException) {
      return 0;
    }  }

  public int getRedYcordinate(){
    blockCount = pixy2.getCCC().getBlocks(false, 3, 255);
    pixy2.getCCC().getBlocks(false, 1, 255);
    blocks = pixy2.getCCC().getBlockCache();
    try {
      return blocks.get(0).getY();
    } catch (Exception ArrayIndexOutOfBoundsException) {
      return 0;
    }  }

    public int getBlueXcordinate(){
      blockCount = pixy2.getCCC().getBlocks(false, 3, 255);
      pixy2.getCCC().getBlocks(false, 2, 255);
      blocks = pixy2.getCCC().getBlockCache();
      try {
        return blocks.get(0).getX();
      } catch (Exception ArrayIndexOutOfBoundsException) {
        return 0;
      }  }
  
    public int getBlueYcordinate(){
      blockCount = pixy2.getCCC().getBlocks(false, 3, 255);
      pixy2.getCCC().getBlocks(false, 2, 255);
      blocks = pixy2.getCCC().getBlockCache();
      try {
        return blocks.get(0).getY();
      } catch (Exception ArrayIndexOutOfBoundsException) {
        return 0;
      }  
    }
  

  public int getBlockCount(){
    return blockCount;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
