package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class Elevator extends SubsystemBase{
    
    SparkMax eleMotor;
    SparkMax armMotor;

    private static Robot   instance;
    private        Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Timer disabledTimer;

    public static final SparkMaxConfig elevator1Config = new SparkMaxConfig();
                
      static {
        elevator1Config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
              }
  
    public static final SparkMaxConfig arm1Config = new SparkMaxConfig();
                
      static {
      arm1Config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
              }

    public double deadbandreturn(double JoystickValue, double DeadbandCutOff) {
        double deadbandreturn;
            if (JoystickValue<DeadbandCutOff&&JoystickValue>(DeadbandCutOff*(-1))){
                deadbandreturn=0; // if less than the deadband cutoff, return 0, if greater than the negative deadband cutoff, return 0
            }
            else{
                deadbandreturn=(JoystickValue- // initially in one of two ranges: [DeadbandCutOff,1] or -1,-DeadBandCutOff]
                (Math.abs(JoystickValue)/JoystickValue // 1 if JoystickValue > 0, -1 if JoystickValue < 0 (abs(x)/x); could use Math.signum(JoystickValue) instead
                *DeadbandCutOff // multiply by the sign so that for >0, it comes out to - (DeadBandCutOff), and for <0 it comes to - (-DeadBandCutOff)
                )
                ) // now in either [0,1-DeadBandCutOff] or -1+DeadBandCutOff,0]
                /(1-DeadbandCutOff); // scale to [0,1] or -1,0]
            }
        return deadbandreturn;
    }

    public Robot()
    {
        instance = this;
    }

    public static Robot getInstance()
    {
        return instance;
    }


    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    public void robotInit()
    {
        eleMotor = new SparkMax(23, MotorType.kBrushless);
        eleMotor.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotor = new SparkMax(25, MotorType.kBrushless);
        armMotor.configure(arm1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
        // immediately when disabled, but then also let it be pushed more 
        disabledTimer = new Timer();
    }


    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    public void disabledInit()
    {
        m_robotContainer.setMotorBrake(true);
        disabledTimer.reset();
        disabledTimer.start();
    }


    public void disabledPeriodic()
    {
        if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
        {
            m_robotContainer.setMotorBrake(false);
            disabledTimer.stop();
            disabledTimer.reset();
        }
    }


    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    public void autonomousInit()
    {
        m_robotContainer.setMotorBrake(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.schedule();
        }
    }
}