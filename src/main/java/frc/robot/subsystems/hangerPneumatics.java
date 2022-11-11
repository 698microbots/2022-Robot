// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hangerPneumatics extends SubsystemBase {
  /** Creates a new hangerPneumatics. */
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  public hangerPneumatics() {
  compressor.close(); //.stop not used anymore
  }

  public void setSolForward(){
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void setSolBackward(){
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void turnComOn(){
    compressor.enableDigital();
  }

  public void turnComOff(){
    compressor.disable();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    solenoid.get();
  }
}
