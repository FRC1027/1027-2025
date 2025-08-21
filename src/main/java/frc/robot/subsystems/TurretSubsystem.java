package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class TurretSubsystem extends SubsystemBase {

  SparkMax turret; // Motor controller for turret mechanism

  public static final SparkMaxConfig turretConfig = new SparkMaxConfig();

    // Configure motor settings: brake mode and current limiting
    static {
        turretConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    }
  
  private static final double MAX_TURRET_SPEED = 0.5; // Max speed [-0.5, 0.5]

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

  public TurretSubsystem() {
    // Initialize intake motor on CAN ID 23, brushless
    turret = new SparkMax(23, MotorType.kBrushless);
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
 /**
  * Uses Limelight vision data to auto-align the turret to fiducial tag 4.
  * If the target is visible, calculates proportional power and clamps output.
  */
  public void trackTargetWithLimelight() {
    double tx = LimelightHelpers.getTX("limelight");         // horizontal offset
    boolean tv = LimelightHelpers.getTV("limelight");        // target valid
    double tid = LimelightHelpers.getFiducialID("limelight"); // tag ID

    if (tv && tid == 4) { // target detected and correct tag
      double kP = 0.02;
      double minCommand = 0.05;
      double turretPower = kP * tx;

      if (Math.abs(tx) > 1.0) { // only move if offset is significant
        double output = turretPower + Math.copySign(minCommand, tx);
        double clampedOutput = MathUtil.clamp(output, -MAX_TURRET_SPEED, MAX_TURRET_SPEED);
        turret.set(clampedOutput);

        System.out.printf("Auto-Aligning: tx=%.2f, tid=%.0f, raw=%.2f, clamped=%.2f%n", tx, tid, output, clampedOutput);
      } else {
        turret.set(0); // small offset → stop
      }
    } else {
      turret.set(0); // no valid target → stop
    }
  }

  /** 
   * Called once per scheduler run. 
   * Allows manual control or auto-aligning depending on trigger input.
   */
  @Override
  public void periodic() {
    double trigger = RobotContainer.mechXbox.getRightTriggerAxis();   // Right trigger
    double turretManualControl = RobotContainer.mechXbox.getLeftX();  // Manual control

    if (trigger > 0.5 && Math.abs(turretManualControl) < 0.1) {
      // Auto-align ONLY if trigger is pressed and driver isn't touching the stick
      trackTargetWithLimelight();
      System.out.println("Auto-aligning to tag 4");
    } else if (Math.abs(turretManualControl) >= 0.1) {
      // Manual control if joystick is moved
      turret.set(deadbandreturn(turretManualControl, 0.1));
    } else {
      // No input → stop turret
      turret.set(0);
    }
  }

  /** Periodically called during simulation (currently unused). */
  @Override
  public void simulationPeriodic() {
  
  }

  /** Manual turret control, applies deadband */
  public void manualControl(double input) {
    turret.set(deadbandreturn(input, 0.1));
  }

  /** Command that auto-tracks if a_val == 1, stops turret otherwise. */
  public Command a_trackerTurretAuto() {
    return run(() -> {
        if (RobotContainer.a_val == 1) {
            trackTargetWithLimelight();
        } else {
          turret.set(0);
        }
    });
  }
}