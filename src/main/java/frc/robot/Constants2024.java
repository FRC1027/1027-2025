package frc.robot;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
//import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSFalconSwerveConstants;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

public final class Constants2024 {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {

        // Spark Max Idle Modes
        // public static final CANSparkMax.IdleMode driveIdleMode = CANSparkMax.IdleMode.kCoast;
        // public static final CANSparkMax.IdleMode angleIdleMode = CANSparkMax.IdleMode.kCoast;
        public static final SparkMaxConfig.IdleMode driveIdleMode = SparkMaxConfig.IdleMode.kCoast; //ML Drycoded must test
        public static final SparkMaxConfig.IdleMode angleIdleMode = SparkMaxConfig.IdleMode.kCoast; //ML Drycoded must test

        // Max Output Powers
        public static final double drivePower = 1;
        public static final double anglePower = .9;


        
        // Gyro
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        // Swerve Module Type
        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
                .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);
        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final double angleGearRatio = chosenModule.angleGearRatio;
        // the number of degrees that a single rotation of the turn motor turns the
        // wheel.
        public static final double DegreesPerTurnRotation = 360 / angleGearRatio;
        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;

        // encoder setup
        // meters per rotation
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        
        public static final double driveRevToMeters = wheelCircumference / (driveGearRatio) * 0.94;
        public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60;
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(30);
        public static final double wheelBase = Units.inchesToMeters(30);
        public static final double WheelTurnDistance = 0.7400;       //Measure distance is 0.74m. But could only turn 80 degree
                                                                    // Increase to 0.75m, could achieve 90 degree
        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
        public static final int driveContinuousCurrentLimit = 40;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
       
           /* Drive Motor info */
           public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

           public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * wheelCircumference)
                   / driveGearRatio;
   
        /* Angle Motor PID Values */
        // Update this setting to accomodate smartMotion
        public static final double angleKP = 0.0006;
        public static final double angleKI = 0;
        public static final double angleKD = 0;
        public static final double angleKFF = 0.000156;

        public static double maxAngleVel = 6.0;          // for velocity setting, using converted value m/s
        public static double maxAnglePos = 500;          // for position setting, using raw encoder rpm/m
        public static double minAngleVel = 0;
        public static double maxAngleAccVel= 4.0;          // for velocity setting, using converted value m/ss
        public static double maxAngleAccPos = 500;        // for position setting, using raw encoder rpm/mm
        public static double allowedAngleErrVel = 0.21;
        public static double allowedAngleErrPos = 0.1;
    
        /* Drive Motor PID Values */
        // Create _v and _p for different pid control
        // _v is for velocity control
        // _p is for position control
        public static final double driveKP_v = 0.1;
        public static final double driveKP_p = 0.0005;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.000156; // 1 / kDriveWheelFreeSpeedRps;
        /** Meters per Second */
        public static final double maxSpeed = 20.0; // 3.6576;
        /** Radians per Second */
        public static final double maxAngularVelocity = 12.0; // 5.0;
        public static double angleRampRate = 0;

        public static double maxDriveVel = 14.0;          // for velocity setting, using converted value m/s
        public static double maxDrivePos = 300;          // for position setting, using raw encoder rpm/m
        public static double minVel = 0;
        public static double maxDriveAccVel= 12.0;          // for velocity setting, using converted value m/ss
        public static double maxDriveAccPos = 300;        // for position setting, using raw encoder rpm/mm
        public static double allowedDriveErrVel = 0.21;
        public static double allowedDriveErrPos = 0.1;
        /* CanCoder Constants */
        public static final CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

        // wheel travel distance / degree, used for base rotation
        public static double turnRatio = WheelTurnDistance * Math.PI / 360;
        public static class Modules {
            /* Module Specific Constants */
            /* Front Left Module - Module 0 */
            public static final class Mod0 {

                public static final int driveMotorID = 2;
                public static final int angleMotorID = 1;
                public static final int canCoderID = 12;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(117.24);// 180: 162.24);
                public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset);
            }

            /* Front Right Module - Module 1 */
            public static final class Mod1 {
                public static final int driveMotorID = 3;
                public static final int angleMotorID = 4;
                public static final int canCoderID = 11;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(190.45);// 180:-165.74);;
                public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset);
            }

            /* Back Left Module - Module 2 */
            public static final class Mod2 {
                public static final int driveMotorID = 6;
                public static final int angleMotorID = 5;
                public static final int canCoderID = 13;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(336.88); // 180: 94.02);
                public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset);
            }

            /* Back Right Module - Module 3 */
            public static final class Mod3 {
                public static final int driveMotorID = 8;
                public static final int angleMotorID = 7;
                public static final int canCoderID = 14;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(149.76);
                public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset);
            }
        }
    }

    public static final class CameraConstants {

        public static final double ROLL = -Math.PI / 2;
        public static final double PITCH = 0.0;
        public static final double YAW = 0.0;
        public static final Transform3d KCAMERA_TO_ROBOT = new Transform3d(
                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(8),
                        Units.inchesToMeters(22.125)),
                new Rotation3d(ROLL, PITCH, YAW)).inverse();

        public static final String CAMERA_NAME = "photonvision";
        
        public static final double LARGEST_DISTANCE = 0.1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

        public static final double X_kP = 5;
        public static final double X_kI = 0;
        public static final double X_kD = 0;

        public static final double Y_kP = 5;
        public static final double Y_kI = 0;
        public static final double Y_kD = 0;

        public static final double THETA_kP = 6.2;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0;

        // Motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
    public static class PlatformConstants {
        public static final int kSolenoidForward1 = 6;
        public static final int kSolenoidReverse1 = 7;

        //public static final int kSolenoidForward2 = 2;
        //public static final int kSolenoidReverse2 = 3;

        //public static final int kSolenoidForward3 = 4;
        //public static final int kSolenoidReverse3 = 5;
  }
}
