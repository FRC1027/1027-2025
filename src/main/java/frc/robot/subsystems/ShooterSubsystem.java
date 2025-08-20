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
    
    SparkMax intake; // Motor controller for intake/shooter mechanism

    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

      // Configure motor settings: brake mode and current limiting
      static {
      intakeConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      }

    /**
     * Applies a deadband to joystick input to ignore small unintended movements.
     * If the absolute value of the joystick input is below DeadbandCutOff, returns 0.
     * Otherwise, rescales the remaining range back to [-1, 1].
     *
     * @param JoystickValue the raw joystick input
     * @param DeadbandCutOff the minimum threshold to ignore input
     * @return the adjusted joystick value after applying deadband
     */
    public double deadbandreturn(double JoystickValue, double DeadbandCutOff) {
        double deadbandreturn;
        if (JoystickValue < DeadbandCutOff && JoystickValue > -DeadbandCutOff) {
            // Inside deadband → treat as zero
            deadbandreturn = 0;
        } else {
            // Outside deadband → rescale input
            deadbandreturn = (JoystickValue -
                (Math.abs(JoystickValue) / JoystickValue   // returns sign (+1 or -1)
                * DeadbandCutOff))                        // subtract deadband threshold
                / (1 - DeadbandCutOff);                   // normalize back to [-1,1]
        }
        return deadbandreturn;
    }

    public ShooterSubsystem() {
        // Initialize intake motor on CAN ID 33, brushless
        intake = new SparkMax(33, MotorType.kBrushless);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

  /**
   * Example command factory method.
   * Returns a one-time command tied to this subsystem.
   */
  public Command exampleMethodCommand() {
    return runOnce(
        () -> {
            // Example one-time action here
        });
  }

    /**
     * Example condition method.
     *
     * @return false (placeholder)
     */
  public boolean exampleCondition() {
    return false;
  }

    /**
     * Called once per scheduler run.
     * Reads controller trigger input and drives the intake motor accordingly.
     */
  @Override
  public void periodic() {
    double inVal = RobotContainer.driverXbox.getRightTriggerAxis();
    double outVal = -RobotContainer.driverXbox.getLeftTriggerAxis();

    // Priority order: Outtake > Intake > Stop
    if (RobotContainer.driverXbox.getLeftTriggerAxis() > 0.1) {
        // Outtake (left trigger) if pressed beyond deadband
        intake.set(deadbandreturn(outVal / 4, 0.1));
    } else if (RobotContainer.driverXbox.getRightTriggerAxis() > 0.1) {
        // Intake (right trigger) if pressed beyond deadband
        intake.set(deadbandreturn(inVal / 4, 0.1));
    } else {
        // Neither trigger pressed → stop
        intake.set(0);
    }
  }

  /** Periodically called during simulation (currently unused). */
  @Override
  public void simulationPeriodic() {
  
  }

  /**
     * Runs the intake forward at a fixed speed for 2 seconds, then stops.
     *
     * @return command sequence for timed intake
     */
  public Command twoSecondIntake() {
    return run(() -> {
        intake.set(0.1027);
    }).withTimeout(2)
        .andThen(() -> intake.set(0));
  }

  /**
     * Runs the intake in reverse at a fixed speed for 2 seconds, then stops.
     *
     * @return command sequence for timed outtake
     */
  public Command twoSecondOuttake() {
    return run(() -> {
        intake.set(-0.1027);
    }).withTimeout(2)
        .andThen(() -> intake.set(0));
  }
}