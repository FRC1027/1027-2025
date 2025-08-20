package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
    
    SparkMax intake;

    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

      static {
      intakeConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      }

      /* A method in which any unintended effects from controller deadband is mitigated */
    public double deadbandreturn(double JoystickValue, double DeadbandCutOff) {
        double deadbandreturn;
        if (JoystickValue<DeadbandCutOff && JoystickValue > (DeadbandCutOff * (-1))) {
            deadbandreturn = 0; // if less than the deadband cutoff, return 0, if greater than the negative deadband cutoff, return 0
        }
        else {
            deadbandreturn = (JoystickValue- // initially in one of two ranges: [DeadbandCutOff,1] or -1,-DeadBandCutOff]
            (Math.abs(JoystickValue) / JoystickValue // 1 if JoystickValue > 0, -1 if JoystickValue < 0 (abs(x)/x); could use Math.signum(JoystickValue) instead
            * DeadbandCutOff // multiply by the sign so that for >0, it comes out to - (DeadBandCutOff), and for <0 it comes to - (-DeadBandCutOff)
        )
        ) // now in either [0,1-DeadBandCutOff] or -1+DeadBandCutOff,0]
        /(1 - DeadbandCutOff); // scale to [0,1] or -1,0]
        }
      return deadbandreturn;
    }

    public ShooterSubsystem() {
        intake = new SparkMax(33, MotorType.kBrushless);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  /* This Method Will be Called Once Per Scheduler Run */
  @Override
  public void periodic() {
    double inVal = RobotContainer.driverXbox.getRightTriggerAxis();
    double outVal = -RobotContainer.driverXbox.getLeftTriggerAxis();

    // A series of if statements that check the returned deadband of the intake mechanism
    if (RobotContainer.driverXbox.getLeftTriggerAxis() > 0.1){
        intake.set(deadbandreturn(outVal / 4, 0.1));
    }
      
    if (RobotContainer.driverXbox.getLeftTriggerAxis() < 0.1){
        intake.set(0);
    }
      
    if (RobotContainer.driverXbox.getRightTriggerAxis() > 0.1){
        intake.set(deadbandreturn(inVal / 4, 0.1));
    } 
  }

  /* This Method Will be Called Once Per Scheduler Run During Simulation */
  @Override
  public void simulationPeriodic() {
  
  }

  public Command twoSecondIntake() {
    return run(() -> {
        intake.set(0.1027);
    }).withTimeout(2)
        .andThen(() -> intake.set(0));
  }

  public Command twoSecondOuttake() {
    return run(() -> {
        intake.set(-0.1027);
    }).withTimeout(2)
        .andThen(() -> intake.set(0));
  }
}