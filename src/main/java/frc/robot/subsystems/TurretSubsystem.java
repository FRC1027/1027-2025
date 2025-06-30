// This file defines the behavior of a "turret" subsystem for a robot.
// A turret is a rotating part that can aim at targets.

// These are import statements. They let us use pre-written code (libraries)
// for motors, math functions, commands, and FRC-specific tools.

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

// A "subsystem" in FRC is a part of the robot that can be controlled,
// like a turret, drivetrain, or arm. This one controls the turret.
public class TurretSubsystem extends SubsystemBase {
  
  // Creates a motor controller called "turret" to spin the turret.
  SparkMax turret;

  // This sets up a configuration for the turret motor.
  public static final SparkMaxConfig turretConfig = new SparkMaxConfig();

    // This block runs once and applies some settings to the turret motor:
    static {
        turretConfig
            .idleMode(IdleMode.kBrake)    // When the motor stops, it resists movement (brakes).
            .smartCurrentLimit(50);       // Limits the max current to prevent overheating.
    }

  // The fastest the turret is allowed to turn (in either direction).
  private static final double MAX_TURRET_SPEED = 0.5; // max speed [-0.5, 0.5]


  /**
   * This method removes small, accidental joystick movements (called "deadband").
   * Joysticks aren't perfect, and even when you don't mean to move it, it might report tiny values.
   * This method ignores those small values and scales the rest of the movement.
   */
  public double deadbandreturn(double JoystickValue, double DeadbandCutOff) {
    double deadbandreturn;
      // If joystick value is between -DeadbandCutOff and DeadbandCutOff, treat it as 0
      if (JoystickValue<DeadbandCutOff && JoystickValue > (DeadbandCutOff * (-1))) {
      deadbandreturn = 0; // if less than the deadband cutoff, return 0, if greater than the negative deadband cutoff, return 0
    }
    else {
      // If it's outside the deadband, scale it smoothly to feel natural
      deadbandreturn = (JoystickValue- // initially in one of two ranges: [DeadbandCutOff,1] or -1,-DeadBandCutOff]
      (Math.abs(JoystickValue) / JoystickValue // 1 if JoystickValue > 0, -1 if JoystickValue < 0 (abs(x)/x); could use Math.signum(JoystickValue) instead
       * DeadbandCutOff // multiply by the sign so that for >0, it comes out to - (DeadBandCutOff), and for <0 it comes to - (-DeadBandCutOff)
      )
     ) // now in either [0,1-DeadBandCutOff] or -1+DeadBandCutOff,0]
     /(1 - DeadbandCutOff); // scale to [0,1] or -1,0]
    }
      return deadbandreturn;
    }

  /**
   * Constructor - this runs once when the robot starts.
   * It tells the program which motor port the turret is plugged into (port 23),
   * and what type of motor it is (brushless).
   */
  public TurretSubsystem() {
    turret = new SparkMax(23, MotorType.kBrushless); // Creates motor on port 23
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    /**
   * This command spins the turret at 50% speed while a button is held.
   * When the button is released, it stops.
   */
  public Command spinTurret() {
    return this.startEnd(
        () -> this.turret.set(0.5), () -> this.turret.set(0.0));
  }

  /**
   * Placeholder example for a one-time command.
   * Could be used to do something once, like centering the turret.
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
   * Another example: returns true or false depending on something, like a sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  /**
   * Creates a command that keeps running and tries to aim the turret at a specific tag using a camera.
   * It uses Limelight, a special vision system for robots.
   */
  public Command trackTargetCommand() {
    return run(() -> trackTargetWithLimelight());
  }
  
  /**
   * This function auto-aims the turret using the Limelight camera.
   * It checks if the camera sees a valid target with ID 4.
   * If it does, it rotates the turret to center on that target using basic math.
   */
  public void trackTargetWithLimelight() {
    double tx = LimelightHelpers.getTX("limelight");      // Horizontal offset from crosshair to target
    boolean tv = LimelightHelpers.getTV("limelight");     // True if the camera sees a target
    double tid = LimelightHelpers.getFiducialID("limelight"); // ID of the tag seen (we want ID 4)

    if (tv && tid == 4) { // Only continue if a target is found and it's tag ID 4
      double kP = 0.02;     // Proportional control constant (how much to turn based on error)
      double minCommand = 0.05; // Minimum motor power to overcome friction
      double turretPower = kP * tx; // How hard to turn based on how far off-center the target is
    
      // Only adjust if the target is noticeably off-center
      if (Math.abs(tx) > 1.0) {
        double output = turretPower + Math.copySign(minCommand, tx);
        double clampedOutput = MathUtil.clamp(output, -MAX_TURRET_SPEED, MAX_TURRET_SPEED);
        turret.set(clampedOutput); // Turn the turret
        System.out.printf("Auto-Aligning: tx=%.2f, tid=%.0f, raw=%.2f, clamped=%.2f%n", tx, tid, output, clampedOutput);
      } else {
        turret.set(0); // If already centered, do nothing
      }
    } else {
      turret.set(0); // No target found, stop moving
    }
  }

  /**
   * This method runs repeatedly while the robot is operating.
   * It checks if the driver is holding down the right trigger.
   * If so, it turns on auto-aligning. Otherwise, the driver can control the turret manually.
   */
  @Override
  public void periodic() {
    double trigger = RobotContainer.mechXbox.getRightTriggerAxis(); // How much the right trigger is pressed
    double leftTurret = RobotContainer.mechXbox.getLeftX();         // Left joystick X controls turret manually

    if (trigger > 0.5) {
      // If trigger is pressed past halfway, auto-align to vision target
      trackTargetWithLimelight();
      System.out.println("Auto-aligning to tag 4");
  } else {
      // Otherwise, use joystick to control turret manually (with deadband)
      turret.set(deadbandreturn(leftTurret, 0.1));
  }
}

  /**
   * This method runs repeatedly, but only in simulation (not on the real robot).
   * It can be used to simulate the turret if needed.
   */
  @Override
  public void simulationPeriodic() {
    // You could simulate motor movement here if needed.
  }
}