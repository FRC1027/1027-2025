// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
  /** Creates a new PneumaticsSubsystem. */

  private Compressor compressor;
  //private PneumaticHub pneumaticsHub =new PneumaticHub(1);
  
  public PneumaticsSubsystem() {
    compressor = new Compressor(52, PneumaticsModuleType.CTREPCM);

    System.out.println("===== Activating compressor!! =====");
    activateCompressor();
  }

  public void activateCompressor() {
    compressor.enableDigital();
  }

  public void deactivateCompressor() {
    compressor.disable();
  }

  //public PneumaticHub getPneumaticsHub() {
  //  return pneumaticsHub;
  //}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
