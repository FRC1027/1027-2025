package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class TurretSubsystem extends SubsystemBase {
  // Establishes the turret as a SparkMax Object
  SparkMax turret;

  // Configures the turret motor controls
  public static final SparkMaxConfig turretConfig = new SparkMaxConfig();

    static {
        turretConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    }
  private static final double MAX_TURRET_SPEED = 0.5; // max speed [-0.5, 0.5]


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

public Command a_tracker() {
    return new InstantCommand(() -> {
        if (RobotContainer.a_val == 1) {
            trackTargetWithLimelight();
        }
    });
}


}