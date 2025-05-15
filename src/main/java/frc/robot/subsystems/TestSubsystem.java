package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TestSubsystem extends SubsystemBase {
  // Establishes the turret as a SparkMax Object
  SparkMax turret;
  
  // Creates the mechanism controller
  final public XboxController mechXbox = new XboxController(1);

  // Configures the turret motor controls
  public static final SparkMaxConfig turretConfig = new SparkMaxConfig();

      static {
        turretConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      }
  
  private static Robot   instance;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Timer disabledTimer;

  // A Method were any Unintended Effects from Controller Deadband is Mitigated
  public double deadbandreturn(double JoystickValue, double DeadbandCutOff) {
    double deadbandreturn;
      if (JoystickValue < DeadbandCutOff && JoystickValue > (DeadbandCutOff * (-1))) {
      deadbandreturn = 0; // if less than the deadband cutoff, return 0, if greater than the negative deadband cutoff, return 0
    }
    else {
      deadbandreturn = (JoystickValue - // initially in one of two ranges: [DeadbandCutOff,1] or -1,-DeadBandCutOff]
      (Math.abs(JoystickValue) / JoystickValue // 1 if JoystickValue > 0, -1 if JoystickValue < 0 (abs(x)/x); could use Math.signum(JoystickValue) instead
       * DeadbandCutOff // multiply by the sign so that for >0, it comes out to - (DeadBandCutOff), and for <0 it comes to - (-DeadBandCutOff)
      )
     ) // now in either [0,1-DeadBandCutOff] or -1+DeadBandCutOff,0]
     / (1 - DeadbandCutOff); // scale to [0,1] or -1,0]
    }
      return deadbandreturn;
    }

    turret = new SparkMax(23, MotorType.kBrushless);
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}