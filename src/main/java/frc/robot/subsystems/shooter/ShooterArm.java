package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto_OneNote;

import static frc.robot.Constants.Swerve.maxSpeed;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;

public class ShooterArm extends SubsystemBase{
    //private final TalonSRX motor;
    private final TalonFX motor;
    private final double MAX_SPEED = 0.8; // 100% max speed
    private final double delay = 0.1;
    private final double autoDelay = 0.15;

    public ShooterArm(int motorCANId) {
        this.motor = new TalonFX(motorCANId);
    }

    public void shoot() {
        motor.set(MAX_SPEED);
        //new WaitCommand(delay).schedule();
        Timer.delay(delay);
        motor.stopMotor(); 
        //new WaitCommand(0.1).schedule();
        Timer.delay(0.1);
        //Starts Return sequence
        motor.set(-MAX_SPEED/6);
        //new WaitCommand(delay*5).schedule();
        Timer.delay(delay*4.3);
        motor.stopMotor();
    }

    public void auto_shoot(){
        motor.set(MAX_SPEED);
        new WaitCommand(autoDelay).schedule();
        //Timer.delay(autoDelay);
        motor.stopMotor();
        new WaitCommand(0.1).schedule();
        //Starts return sequence
        //Timer.delay(0.1);
        motor.set(-MAX_SPEED/6);
        new WaitCommand(delay*5).schedule();
        //Timer.delay(delay*5);
        motor.stopMotor();
    }

    public Command shootCommand(){
        //return this.runOnce(()-> auto_shoot());
        return new InstantCommand(this::auto_shoot);
    }
    public void stopShooter(){
        motor.stopMotor();
    }
}
