// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;


// public class Elevator extends SubsystemBase{
    
//     final public XboxController mechXbox = new XboxController(1);

//     SparkMax eleMotor1;
//     SparkMax eleMotor2;
//     SparkMax armMotor1;
//     SparkMax armMotor2;
//     SparkMax intake;

//     private static Robot   instance;
//     private        Command m_autonomousCommand;
//     private RobotContainer m_robotContainer;
//     private Timer disabledTimer;


//     /*
//      * Configuration for Elevator and Arm Motors
//      */
//     public static final SparkMaxConfig elevator1Config = new SparkMaxConfig();

//     public static final SparkMaxConfig elevator2Config = new SparkMaxConfig();
                
//       static {
//         elevator1Config
//           .idleMode(IdleMode.kBrake)
//           .smartCurrentLimit(50);
//               }

//       static {
//         elevator2Config
//           .idleMode(IdleMode.kBrake)
//           .smartCurrentLimit(50);
//               }
  
//     public static final SparkMaxConfig arm1Config = new SparkMaxConfig();

//     public static final SparkMaxConfig arm2Config = new SparkMaxConfig();
                
//       static {
//         arm1Config
//           .idleMode(IdleMode.kBrake)
//           .smartCurrentLimit(50);
//               }

//       static {
//         arm2Config
//           .idleMode(IdleMode.kBrake)
//           .smartCurrentLimit(50);
//               }

//    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

//      static {
//        intakeConfig
//          .idleMode(IdleMode.kBrake)
//          .smartCurrentLimit(50);
//              }

    
//     /*
//      * Controller Deadband Auto Calibrator
//      */
//     public double deadbandreturn(double JoystickValue, double DeadbandCutOff) {
//         double deadbandreturn;
//             if (JoystickValue<DeadbandCutOff&&JoystickValue>(DeadbandCutOff*(-1))){
//                 deadbandreturn=0; // if less than the deadband cutoff, return 0, if greater than the negative deadband cutoff, return 0
//             }
//             else{
//                 deadbandreturn=(JoystickValue- // initially in one of two ranges: [DeadbandCutOff,1] or -1,-DeadBandCutOff]
//                 (Math.abs(JoystickValue)/JoystickValue // 1 if JoystickValue > 0, -1 if JoystickValue < 0 (abs(x)/x); could use Math.signum(JoystickValue) instead
//                 *DeadbandCutOff // multiply by the sign so that for >0, it comes out to - (DeadBandCutOff), and for <0 it comes to - (-DeadBandCutOff)
//                 )
//                 ) // now in either [0,1-DeadBandCutOff] or -1+DeadBandCutOff,0]
//                 /(1-DeadbandCutOff); // scale to [0,1] or -1,0]
//             }
//         return deadbandreturn;
//     }


//     public static Robot getInstance()
//     {
//         return instance;
//     }


//     /**
//      * This function is run when the robot is first started up and should be used for any initialization code.
//      */
//     public void robotInit()
//     {
//         eleMotor1 = new SparkMax(23, MotorType.kBrushless);
//         eleMotor2 = new SparkMax(0, MotorType.kBrushless);
//         eleMotor1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         eleMotor2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         armMotor1 = new SparkMax(25, MotorType.kBrushless);
//         armMotor2 = new SparkMax(0, MotorType.kBrushless);
//         armMotor1.configure(arm1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         armMotor2.configure(arm2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         intake = new SparkMax(28, MotorType.kBrushless);
//         intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//         // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
//         // autonomous chooser on the dashboard.
//         m_robotContainer = new RobotContainer();

//         // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
//         // immediately when disabled, but then also let it be pushed more 
//         disabledTimer = new Timer();
//     }


//     /**
//      * This function is called once each time the robot enters Disabled mode.
//      */
//     public void disabledInit()
//     {
//         m_robotContainer.setMotorBrake(true);
//         disabledTimer.reset();
//         disabledTimer.start();
//     }


//     public void disabledPeriodic()
//     {
//         if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
//         {
//             m_robotContainer.setMotorBrake(false);
//             disabledTimer.stop();
//             disabledTimer.reset();
//         }
//     }


//     /**
//      * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
//      */
//     public void autonomousInit()
//     {
//         m_robotContainer.setMotorBrake(true);
//         m_autonomousCommand = m_robotContainer.getAutonomousCommand();

//         // schedule the autonomous command (example)
//         if (m_autonomousCommand != null)
//         {
//             m_autonomousCommand.schedule();
//         }
//     }


//     public void teleopInit()
//     {
//         // This makes sure that the autonomous stops running when
//         // teleop starts running. If you want the autonomous to
//         // continue until interrupted by another command, remove
//         // this line or comment it out.
//         if (m_autonomousCommand != null)
//         {
//         m_autonomousCommand.cancel();
//         } else
//         {
//         CommandScheduler.getInstance().cancelAll();
//         }
//     }


//     /**
//      * This function is called periodically during operator control.
//      */
//     public void teleopPeriodic()
//     {
//         double upElevator = -mechXbox.getLeftY();
//         double forwardArm = -mechXbox.getRightY();
//         double inVal = mechXbox.getRightTriggerAxis();
//         double outVal = mechXbox.getLeftTriggerAxis();
//         eleMotor1.set(-deadbandreturn(upElevator, 0.1));
//         eleMotor2.set(deadbandreturn(upElevator, 0.1));
//         armMotor1.set(-deadbandreturn(forwardArm, 0.1));
//         armMotor2.set(-deadbandreturn(forwardArm, 0.1));
//         intake.set(deadbandreturn(inVal, 0.1));
//         intake.set(deadbandreturn(outVal, 0.1));
//     }
// }