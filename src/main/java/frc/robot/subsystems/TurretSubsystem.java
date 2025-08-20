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

  // Configures the turret motor controls
  public static final SparkMaxConfig turretConfig = new SparkMaxConfig();

    static {
        turretConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    }
  
  private static final double MAX_TURRET_SPEED = 0.5; // max speed [-0.5, 0.5]

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

  /** Creates the TurretSubsystem. */
  public TurretSubsystem() {
    turret = new SparkMax(23, MotorType.kBrushless);
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command spinTurret() {
      return this.startEnd(
          () -> this.turret.set(0.5), () -> this.turret.set(0.0));
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

  public Command trackTargetCommand() {
        return run(() -> trackTargetWithLimelight());
  }
  
  public void trackTargetWithLimelight() {
    double tx = LimelightHelpers.getTX("limelight");
    boolean tv = LimelightHelpers.getTV("limelight");
    double tid = LimelightHelpers.getFiducialID("limelight");

    if (tv && tid == 4) {
      double kP = 0.02;
      double minCommand = 0.05;
      double turretPower = kP * tx;
    
      if (Math.abs(tx) > 1.0) {
        double output = turretPower + Math.copySign(minCommand, tx);
        double clampedOutput = MathUtil.clamp(output, -MAX_TURRET_SPEED, MAX_TURRET_SPEED);
        turret.set(clampedOutput);
        System.out.printf("Auto-Aligning: tx=%.2f, tid=%.0f, raw=%.2f, clamped=%.2f%n", tx, tid, output, clampedOutput);
      } else {
        turret.set(0);
      }
    } else {
      turret.set(0);
    }
  }

  /* This Method Will be Called Once Per Scheduler Run */
  //@Override
  //public void periodic() {
  //  double leftTurret = RobotContainer.mechXbox.getLeftX();
  //  turret.set(deadbandreturn(leftTurret, 0.1));
  //}
  @Override
  public void periodic() {
    double trigger = RobotContainer.mechXbox.getRightTriggerAxis(); // Right trigger
    double leftTurret = RobotContainer.mechXbox.getLeftX();         // Manual control

    if (trigger > 0.5) {
      trackTargetWithLimelight(); // Auto-align if trigger held
      System.out.println("Auto-aligning to tag 4");
  } else {
      turret.set(deadbandreturn(leftTurret, 0.1)); // Manual control
  }
}

  /* This Method Will be Called Once Per Scheduler Run During Simulation */
  @Override
  public void simulationPeriodic() {
  
  }

public void manualControl(double input) {
  turret.set(deadbandreturn(input, 0.1));
}

public Command a_tracker() {
    return run(() -> {
        if (RobotContainer.a_val == 1) {
            trackTargetWithLimelight();
        } else {
          turret.set(0);
        }
    });
}
}