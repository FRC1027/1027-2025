// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  /**
   *  Variables for automatic April Tag alignment with PhotonVision
   */
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(8);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(8.5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("photonvision");

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.0005;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.0012;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);


  SparkMax eleMotor;
  SparkMax armMotor;
    
  // Creates a second controller
   //final public        CommandXboxController mechXbox = new CommandXboxController(1);
   final public XboxController mechXbox = new XboxController(1);

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

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public double deadbandreturn(double JoystickValue, double DeadbandCutOff) {
    double deadbandreturn;
        if (JoystickValue<DeadbandCutOff&&JoystickValue>(DeadbandCutOff*(-1))) {
        deadbandreturn=0; // if less than the deadband cutoff, return 0, if greater than the negative deadband cutoff, return 0
    }
    else {
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
  @Override
  public void robotInit()
  {
      eleMotor = new SparkMax(23, MotorType.kBrushless);
      eleMotor.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      armMotor = new SparkMax(25, MotorType.kBrushless);
      armMotor.configure(arm1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // Adds the camera feed of our photonvision/limelight cameras to the SmartDashboard as defined in Vision.java  
    CameraServer.startAutomaticCapture("photonvision", 0);
    


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
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
  @Override
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

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    double upElevator = -mechXbox.getLeftY();
    double forwardArm = -mechXbox.getRightY();
    eleMotor.set(-deadbandreturn(upElevator, 0.1));
    armMotor.set(-deadbandreturn(forwardArm, 0.1)/6);


    /**
     *  Automatic alignment to April Tags using PhotonVision
     */
    double rotationSpeed;

    if (mechXbox.getAButton()) {

      System.out.println("A Button Works!");

      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();

      if (result.hasTargets()) {

        System.out.println("Target Detected!");

        // Calculate angular turn power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
      } else {
        // If we have no targets, stay still.
        rotationSpeed = 0;
      }
    } else {
      // Manual Driver Mode
      rotationSpeed = mechXbox.getLeftX();
    }
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
