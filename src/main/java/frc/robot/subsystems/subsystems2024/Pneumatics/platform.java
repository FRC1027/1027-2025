// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subsystems2024.Pneumatics;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants2024;

//import java.util.function.DoubleSupplier;

public class platform extends SubsystemBase {

  private final DoubleSolenoid solenoid1 = new DoubleSolenoid(52,PneumaticsModuleType.CTREPCM, Constants2024.PlatformConstants.kSolenoidForward1, Constants2024.PlatformConstants.kSolenoidReverse1);
  //private final DoubleSolenoid solenoid2 = new DoubleSolenoid(1,PneumaticsModuleType.CTREPCM, Constants2024.IntakeGripperConstants.kSolenoidForward2, Constants2024.IntakeGripperConstants.kSolenoidReverse2);
  //private final DoubleSolenoid solenoid3 = new DoubleSolenoid(1,PneumaticsModuleType.CTREPCM, Constants2024.IntakeGripperConstants.kSolenoidForward3, Constants2024.IntakeGripperConstants.kSolenoidReverse3);
  /** Constructs a MecanumDrive and resets the gyro. */
  public platform() {

  }

  public void setClampedSolenoid1(boolean clamped) {
    System.out.println("C:"+clamped);
    solenoid1.set(clamped ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }
/*
  public void setClampedSolenoid2(boolean clamped) {
    System.out.println("C:"+clamped);
    solenoid2.set(clamped ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public void setClampedSolenoid3(boolean clamped) {
    System.out.println("C:"+clamped);
    solenoid3.set(clamped ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }
*/
  //public Command setClampedCommand(boolean clamped) {
  //  return run(() -> setClampedSolenoid1(clamped));
  //}
}
