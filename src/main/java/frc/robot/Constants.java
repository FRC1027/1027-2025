// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  private Constants() {} // Prevent instantiation

  /* ================= Robot Physical Properties ================= */

  /** Robot mass in kilograms (measured weight minus bumpers). */
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // lbs → kg

  /** Center of mass used for swerve dynamics calculations. */
  public static final Matter CHASSIS =
      new Matter(
          new Translation3d(0, 0, Units.inchesToMeters(8)),
          ROBOT_MASS);

  /** Control loop time (20ms driver station + 110ms Spark MAX latency). */
  public static final double LOOP_TIME = 0.13; //sec

  /** Maximum linear speed of the robot (m/s). */
  public static final double MAX_SPEED = Units.feetToMeters(14.5);


  /* ================= Shooter ================= */

  /** Motor power used for shooting. */
  public static final double SHOOTER_POWER = 0.1027;

  /** Duration to run shooter for intake/outtake (seconds). */
  public static final double SHOOTER_TIME = 2.0;


  /* ================= Turret ================= */

  /** Maximum turret speed (percent output). */
  public static final double MAX_TURRET_SPEED = 0.5; // Max speed [-0.5, 0.5]


  /* ================= Drivebase ================= */

  public static final class DrivebaseConstants {

    private DrivebaseConstants() {} // Prevent instantiation

    /** Time to hold wheel lock when disabled (seconds). */
    public static final double WHEEL_LOCK_TIME = 10.0;
  }


  /* ================= Operator Controls ================= */

  public static class OperatorConstants {

    private OperatorConstants() {} // Prevent instantiation

    /** Joystick deadband to prevent drift. */
    public static final double DEADBAND = 0.1;

    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;

    /** Scalar applied to turning input. */
    public static final double TURN_CONSTANT = 6.0;
  }


  /* ================= Elevator ================= */

  public static final class ElevatorConstants {

    private ElevatorConstants() {} // Prevent instantiation

    /** Spark MAX configuration for the elevator motor. */
    public static final SparkMaxConfig ELEVATOR_MOTOR_CONFIG = new SparkMaxConfig();
                
    static {
      ELEVATOR_MOTOR_CONFIG
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60);

        ELEVATOR_MOTOR_CONFIG.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.1, 0.0, 0.0)
        .outputRange(-1, 1);
    }
  }
}