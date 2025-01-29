package frc.robot.subsystems.subsystems2024.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ShooterIntake extends SubsystemBase{
    private final XboxController operator;
    private final TalonSRX motor;
    private double MAX_SPEED = 0.8;


    public ShooterIntake(int motorCANId, XboxController controller) {
        this.motor = new TalonSRX(motorCANId);
        this.operator = controller;
    }

    public void Intake() {
        double speed = 0.8;
        if(operator.getLeftTriggerAxis()>=0.5 || operator.getRightTriggerAxis()>=0.5){
            speed = MAX_SPEED/3;
        }
        motor.set(ControlMode.PercentOutput, -speed);
        System.out.print("INTAKE");
    }

    public Command Auto_Intake_Command(){
        return new InstantCommand(this::Intake);
    }

    public void Outtake() {
        motor.set(ControlMode.PercentOutput, MAX_SPEED);
        System.out.print("OUTTAKE");
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }
}
