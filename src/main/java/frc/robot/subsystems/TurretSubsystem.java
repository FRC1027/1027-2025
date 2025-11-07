package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.util.Utils;
import frc.robot.Constants;

/**
 * Subsystem that controls the turret motor, supporting both manual and
 * Limelight-based auto-tracking modes.
 */
public class TurretSubsystem extends SubsystemBase {

  SparkMax turret; // Motor controller for turret

  // Shared motor configuration for all turret instances
  public static final SparkMaxConfig turretConfig = new SparkMaxConfig();
    static {
        turretConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    }
  
  /** Creates a new TurretSubsystem with preconfigured SparkMax motor on CAN ID 23 */
  public TurretSubsystem() {
    turret = new SparkMax(23, MotorType.kBrushless);
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  /**
   * Uses Limelight vision data to automatically align the turret to fiducial tag 4.
   * If the target is visible, calculates proportional motor output and clamps it.
   */
  public void trackTargetWithLimelight() {
    double tx = LimelightHelpers.getTX("limelight");          // Horizontal offset
    boolean tv = LimelightHelpers.getTV("limelight");         // Target visible
    double tid = LimelightHelpers.getFiducialID("limelight"); // Target tag ID

    // Only track if correct tag is seen
    if (tv && tid == 4) {
      double kP = 0.02;             // Proportional constant
      double minCommand = 0.05;     // Minimum correction threshold
      double turretPower = kP * tx; // Calculate motor power based on horizontal offset and Proportional constant

      // Only adjust if significantly misaligned
      if (Math.abs(tx) > 1.0) {
        double output = turretPower + Math.copySign(minCommand, tx);
        double clampedOutput = MathUtil.clamp(output, -Constants.MAX_TURRET_SPEED, Constants.MAX_TURRET_SPEED);
        turret.set(clampedOutput);

        System.out.printf("Auto-Aligning: tx=%.2f, tid=%.0f, raw=%.2f, clamped=%.2f%n", tx, tid, output, clampedOutput);
      } else {
        turret.set(0); // Alignment close enough → stop
      }

    } else {
      turret.set(0); // No valid target → hold position
    }
  }

  /**
   * Manual turret control with deadband.
   * @param input joystick input value
   */
  public void manualControl(double input) {
      turret.set(Utils.deadbandReturn(input, 0.1));
  }

  /**
   * Called every scheduler cycle. Switches between auto-tracking and manual control.
   */
  @Override
  public void periodic() {
    double trigger = RobotContainer.mechXbox.getRightTriggerAxis();   // Right trigger that allows for auto-align
    double turretManualControl = RobotContainer.mechXbox.getLeftX();  // Manual control via joystick

    // Auto-align ONLY if trigger pressed and joystick is neutral
    if (trigger > 0.5 && Math.abs(turretManualControl) < 0.1) {
      trackTargetWithLimelight();
    } else if (Math.abs(turretManualControl) >= 0.1) { // Manual override with joystick
      turret.set(Utils.deadbandReturn(turretManualControl, 0.1));
    } else {
      turret.set(0); // No input → stop turret
    }
  }

    /** Optional: Simulation logic (currently unused). */
    @Override
    public void simulationPeriodic() {
      
    }
}