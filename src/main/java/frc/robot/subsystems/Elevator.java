package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private SparkMax elevatorMotor1;
    private PIDController elevatorPIDController;
    private double elevatorEncoderOffset = 0;
    private TrapezoidProfile profile;
    private double elevatorPosSetpoint;
    private double elevatorPower;
    private Timer elevatorProfileTimer;
    private boolean elevatorManual;
    private State startState;
    private State currentState;
    

    public Elevator() {
        this.elevatorMotor1 = new SparkMax(10, MotorType.kBrushless);
        elevatorMotor1.configure(Constants.Elevator.elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorPIDController = new PIDController(0.0001, 0, 0);
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(180, 240));
        elevatorPosSetpoint = 0;
        elevatorPower = 0;
        elevatorProfileTimer = new Timer();
        elevatorManual = true;
        startState = new State(0, 0);
        currentState = new State(0, 0);
    }

    public void setElevatorSpeed(double speed) {
        elevatorPower = speed;
        elevatorManual = true;
    }

    public void setElevatorPosition(double position) {
        if(elevatorManual) {
            startState = new State(elevatorMotor1.getEncoder().getPosition(), elevatorMotor1.getEncoder().getVelocity()/60);
        }
        else {
            startState = new State(currentState.position, currentState.velocity);
        }
        elevatorPosSetpoint = position;
        elevatorManual = false;
        elevatorProfileTimer.restart();
    }

    public void stopElevator() {
        elevatorPower = 0;
        elevatorManual = true;
    }
    @Override
    public void periodic(){
        if(elevatorManual) {
            elevatorMotor1.set(elevatorPower);
        }
        else {
            currentState = profile.calculate(
                elevatorProfileTimer.get(),
                startState,
                new State(elevatorPosSetpoint, 0)
            );
            SmartDashboard.putNumber("Elevator current", getElevatorPosition());
            SmartDashboard.putNumber("Elevator calc", currentState.position);
            SmartDashboard.putNumber("Elevator vel", elevatorMotor1.getEncoder().getVelocity()/60);
            SmartDashboard.putNumber("Elevator vel calc", currentState.velocity);
            elevatorMotor1.set(elevatorPIDController.calculate(getElevatorPosition(), currentState.position));
        }
    }

    public PIDController getPIDController() {
        return elevatorPIDController;
    }

    public double getElevatorPosition() {
        return elevatorMotor1.getEncoder().getPosition() + elevatorEncoderOffset;
        
    }

    public double getElevatorCurrent() {
        return elevatorMotor1.getOutputCurrent();
    }

    public void setElevatorEncoderOffset(double offset) {
        elevatorEncoderOffset = offset;
    }

    public void setElevatorEncoderPosition(double position) {
        elevatorEncoderOffset = position - elevatorMotor1.getEncoder().getPosition();
    }
    
}