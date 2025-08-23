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
import frc.robot.util.Utils;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    
    SparkMax intake; // Motor controller for intake/shooter mechanism

    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    // Configure motor settings: brake mode and current limiting
    static {
        intakeConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
    }

    public ShooterSubsystem() {
        // Initialize intake motor on CAN ID 33, brushless
        intake = new SparkMax(33, MotorType.kBrushless);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        intake.set(Utils.deadbandReturn(outVal / 4, 0.1));
    } else if (RobotContainer.driverXbox.getRightTriggerAxis() > 0.1) {
        // Intake (right trigger) if pressed beyond deadband
        intake.set(Utils.deadbandReturn(inVal / 4, 0.1));
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
  public Command TimedIntake() {
    return run(() -> {
        intake.set(Constants.SHOOTER_POWER);
    }).withTimeout(Constants.SHOOTER_TIME)
        .andThen(() -> intake.set(0));
  }

  /**
     * Runs the intake in reverse at a fixed speed for 2 seconds, then stops.
     *
     * @return command sequence for timed outtake
     */
  public Command TimedOuttake() {
    return run(() -> {
        intake.set(Constants.SHOOTER_POWER * -1); // Reverse intake for outtake
    }).withTimeout(Constants.SHOOTER_TIME)
        .andThen(() -> intake.set(0));
  }
}