// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  SparkMax eleMotor1;  //Black side
  SparkMax eleMotor2;  //white side
  SparkMax armMotor1;
  SparkMax armMotor2;
  SparkMax intake;
  
  // Creates a second controller
  //final public        CommandXboxController mechXbox = new CommandXboxController(1);
  final public XboxController mechXbox = new XboxController(1);
   

  public static final SparkMaxConfig elevator1Config = new SparkMaxConfig();

  public static final SparkMaxConfig elevator2Config = new SparkMaxConfig();
                
      static {
        elevator1Config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
              }

      static {
        elevator2Config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
             }
  
  public static final SparkMaxConfig arm1Config = new SparkMaxConfig();

  public static final SparkMaxConfig arm2Config = new SparkMaxConfig();
                
      static {
      arm1Config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
              }

      static {
      arm2Config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      }

  public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

      static {
      intakeConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      }

  //SparkClosedLoopController m_ele1Controller = eleMotor1.getClosedLoopController();
  //SparkClosedLoopController m_ele2COntroller = eleMotor2.getClosedLoopController();

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
      eleMotor1 = new SparkMax(25, MotorType.kBrushless);
      eleMotor2 = new SparkMax(27, MotorType.kBrushless);
      eleMotor1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      eleMotor2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      armMotor1 = new SparkMax(23, MotorType.kBrushless);
      armMotor2 = new SparkMax(29, MotorType.kBrushless);
      armMotor1.configure(arm1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      armMotor2.configure(arm2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      intake = new SparkMax(28, MotorType.kBrushless);
      intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // Adds the camera feed of our photonvision/limelight cameras to the SmartDashboard as defined in Vision.java  
    CameraServer.startAutomaticCapture("photonvision1", 0);
    
    CameraServer.startAutomaticCapture("photonvision2", 1);


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
    double inVal = mechXbox.getRightTriggerAxis();
    double outVal = -mechXbox.getLeftTriggerAxis();
    eleMotor1.set(-deadbandreturn(upElevator, 0.1));

    //m_ele1Controller.setReference(10, ControlType.kPosition);

    eleMotor2.set(deadbandreturn(upElevator, 0.1));
    armMotor1.set(deadbandreturn(forwardArm, 0.1));
    armMotor2.set(-deadbandreturn(forwardArm, 0.1));
    
    if (mechXbox.getLeftTriggerAxis() > 0.1){
      intake.set(deadbandreturn(outVal, 0.1));
    }

    if (mechXbox.getRightTriggerAxis() > 0.1){
      intake.set(deadbandreturn(inVal, 0.1));
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
