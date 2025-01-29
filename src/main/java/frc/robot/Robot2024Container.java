package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.hal.simulation.CTREPCMDataJNI;
//import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.CTREPCMSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.commands2024.*;
import frc.robot.subsystems.subsystems2024.shooter.ShooterArm;
import frc.robot.subsystems.subsystems2024.shooter.ShooterIntake;
import frc.robot.subsystems.subsystems2024.swerve.SwerveBase;
import frc.robot.subsystems.subsystems2024.LimeLight.LLDashBoard;
import frc.robot.subsystems.subsystems2024.Pneumatics.PneumaticsSubsystem;
import frc.robot.subsystems.subsystems2024.Pneumatics.platform;

//HelloIan

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class Robot2024Container {
    /* Shuffleboard */
    public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    /* Controllers */
    public final XboxController driver = new XboxController(0);
    public final XboxController operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);
    //private final JoystickButton autoMove = new JoystickButton(driver, XboxController.Button.kB.value);

    //Base Left motorCANId: 30
    //Base Right motorCANId: 31
    //Intake motorCANId: 32
    //Shooter motorCANId: 33

    //ShooterPlatform Setup
    //ShooterPlatform base = new ShooterPlatform(30, 31, driver);
    //////ShooterIntake intakeobj = new ShooterIntake(30);
    public ShooterIntake intakeobj = new ShooterIntake(32,operator);
    //private final Command intake = Commands.run(()-> intakeobj.Intake());
    //ShooterIntake outtakeobj = new ShooterIntake(30);
    //private final Command outtake = Commands.run(()-> outtakeobj.Outtake());
    //ShooterPlatform upobj = new ShooterPlatform(30, 31);
    //ShooterPlatform downobj = new ShooterPlatform(30, 31);
    public static ShooterArm shooter = new ShooterArm(39);
    
    //private final JoystickButton cameraDriveMove = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton angleDriveMove = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton AButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton alignButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton BButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton XButton    = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton YButton    = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton StartButton    = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton BackButton    = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton LeftBumper    = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton RightBumper    = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    

    /* Subsystems */
    
    private final SwerveBase s_Swerve = new SwerveBase();
    
// garlic bread is the best bread !!

    public static final PneumaticsSubsystem pneumaticSubsystem = new PneumaticsSubsystem();
    public static final platform platform = new platform();
    public final LLDashBoard limelight;

    /* Commands */

        // Testing Use of Translation2d and Rotation2d with FieldOrientation
        private Translation2d velocity = new Translation2d(0.5, new Rotation2d(Math.PI*(0)));
        private Rotation2d angularVelocity = new Rotation2d(Math.PI/2);
        private Rotation2d angleOfRobot = new Rotation2d(0); // Angle of the robot must be dependent on the gyro's angle to be field oriented at all times
        private boolean fieldOriented = true;

    private Translation2d spot1 = new Translation2d();
    private Translation2d spot2 = new Translation2d(1, 1);
    /* Robin */
    // Use Translation2D & Rotation 2D and Kinematics method to calculation the movement
    private final Command m_driveSmartPositionPoint = Commands.runOnce(()->s_Swerve.setSmartPositionPoint(spot2, spot1, 1, new Rotation2d()));
    private final Command m_driveSpeed = Commands.runOnce(()->s_Swerve.setDriveSpeed(velocity, angularVelocity, angleOfRobot, fieldOriented));
    private final Command m_driveHeading = Commands.runOnce(()->s_Swerve.setDriveHeading(new Rotation2d(Math.PI/4)));
    /* Yan Hongtao */
    // Use manual calculate input for movement
    private final Command m_driveSmartPosition = Commands.runOnce(()->s_Swerve.setSmartAngle(90));
    private final Command m_driveSmartDirection = Commands.runOnce(()->s_Swerve.setSmartPosition());




    //example of auto move
    DriveToPoseCommand autoMoveCommand = new DriveToPoseCommand(
            s_Swerve,
            s_Swerve::getPose,
            new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
            false
    );

    AngleDriveCommand autoAngleDrive = new AngleDriveCommand(
        s_Swerve,
        s_Swerve::getPose
    );

    CameraDriveCommand autoCameraDrive = new CameraDriveCommand(
        s_Swerve, 
        s_Swerve::getPose
    );

    //List of availble Auto's
    Auto_DoNothing DoNothing = new Auto_DoNothing();
    //InstantCommand ShootOnly = new InstantCommand(() -> shooter.auto_shoot());
    //Command ShootOnly = shooter.shootCommand();
    Auto_ShootOnly ShootOnly = new Auto_ShootOnly();
    Auto_OneNote onenote = new Auto_OneNote(s_Swerve);
    Auto_TwoNote twonote = new Auto_TwoNote(s_Swerve);
    
    //Hello people of the code team!

    /* Network Tables Elements */

    //SendableChooser<Command> movementChooser = new SendableChooser<Command>();
    SendableChooser<String> AutonomousChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public Robot2024Container() {
       // SmartDashboard.putBoolean("auto driving", false);
       DriverStation.reportWarning("s_Swerve:setDefaultCommand PRE established", false);
       s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> driver.getRawAxis(rotationAxis),
                () -> driver.getRawButtonPressed(XboxController.Button.kY.value),
                () -> true
            )
        );
        System.out.println("s_Swerve:setDefaultCommand established");
        DriverStation.reportWarning("s_Swerve:setDefaultCommand established", false);

        /* Auto */
        //PathPlannerServer.startServer(5811);
        //movementChooser.setDefaultOption("taxi", new Taxi(s_Swerve));
        limelight = new LLDashBoard();
        //movementChooser.addOption("Nothing", new InstantCommand());
        AutonomousChooser = new SendableChooser<>();
        AutonomousChooser.setDefaultOption("Nothing", "nothing");
        AutonomousChooser.addOption("(FAIL) Shoot Only", "ShootOnly");
        //movementChooser.addOption("Taxi", autoMoveCommand);
        AutonomousChooser.addOption("Taxi", "autoMoveCommand");
        //movementChooser.addOption("Shoot", Shoot_Auto);
        AutonomousChooser.addOption("(FAIL) One Note", "onenote");
        AutonomousChooser.addOption("(FAIL) Two Note", "twonote");
        //movementChooser.addOption("autoAngleDrive", autoAngleDrive);
        //movementChooser.addOption("autoCameraDrive", autoAngleDrive);
        //CameraServer.startAutomaticCapture("limelightfeed","http://limelight.local:5800");
        
       //Shuffleboard.getTab("Drive Time").addDouble("Battery Voltage", );

        HttpCamera LLFeed = new HttpCamera("LimeLight Feed", "http://limelight.local:5800/stream.mjpg");
        HttpCamera PhotonCamera = new HttpCamera("PhotonVision Feed", "http://10.10.27.11:5800/");
        LLFeed.setFPS(15);
        PhotonCamera.setFPS(20);

        Shuffleboard.getTab("Drive Time").add(LLFeed);
        Shuffleboard.getTab("Drive Time").add(PhotonCamera);

        Shuffleboard.getTab("Drive Time").add("Autonomous Chooser", AutonomousChooser);
        //Shuffleboard Game Time Data
        //SmartDashboard.putData("Movement", movementChooser);

        /* Networking */
        //PhotonVision
        PortForwarder.add(5800, "10.10.27.11", 5800);
        //Limelight Vision
        PortForwarder.add(5800, "10.10.27.10", 5800);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        
        //alignButton.onTrue(new AlignWithAprilTagCommand(s_Swerve,limelight));
        //cameraDriveMove.onTrue(new CameraDriveCommand(s_Swerve, s_Swerve::getPose));
        //angleDriveMove.onTrue(new AngleDriveCommand(s_Swerve, s_Swerve::getPose));

        // Kinematics method
        //BackButton.onTrue(m_driveHeading);
        //YButton.onTrue(m_driveSmartPositionPoint);

        // Manual method
        //StartButton.onTrue(m_driveSmartDirection);
        //BackButton.onTrue(m_driveSmartPosition);
        StartButton.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        //example of auto move
        //autoMove.whileTrue(autoMoveCommand);
        //autoMove.toggleOnFalse(new InstantCommand(() -> autoMoveCommand.cancel()));

        //LeftTrigger.onTrue(intake);
        //RightTrigger.onTrue(outtake);
        
        //Test Phase commands INTAKE
        LeftBumper.onTrue(new InstantCommand(() -> intakeobj.Intake()));
        LeftBumper.onFalse(new InstantCommand(() -> intakeobj.stop()));
        RightBumper.onTrue(new InstantCommand(() -> intakeobj.Outtake()));
        RightBumper.onFalse(new InstantCommand(() -> intakeobj.stop()));
        
        //Test Phase commands SHOOTER
        AButton.onTrue(new InstantCommand(() -> shooter.shoot()));
        //BButton.onTrue(new InstantCommand(() -> shooter.shootForce()));
        //BButton.onFalse(new InstantCommand(() -> shooter.stopShooter()));
        //BButton.onTrue(m_driveHeading);
        
        //DRY CODED!!!!!
        //Test Phrase commands PLATFORM
        YButton.onTrue(new HighRowCommand());
        XButton.onTrue(new FetalPositionCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /*
    public Command getAutonomousCommand() {
        return autoMoveCommand;
    }
    */
    public Command getAutonomousCommand(){
    
        String selectedRoutine = AutonomousChooser.getSelected();
        System.out.println("-------" + selectedRoutine + "-------");
        Command autonomousCommand = null;

        if(selectedRoutine.equals("nothing")){
            autonomousCommand = DoNothing;
        }
        else if(selectedRoutine.equals("ShootOnly")){
            autonomousCommand = ShootOnly;
        }
        else if(selectedRoutine.equals("onenote")){
            autonomousCommand = onenote;
        }
        else if(selectedRoutine.equals("autoMoveCommand")){
            autonomousCommand = autoMoveCommand;
        }
        else if(selectedRoutine.equals("twonote")){
            autonomousCommand = twonote;
        }
        else{
            System.out.println("Error: Not picking up any autonomous chooser options");
            autonomousCommand = DoNothing;
        }
        return autonomousCommand;
    }

    public SwerveBase getSwerveBase() {
        return s_Swerve;
    }
}
