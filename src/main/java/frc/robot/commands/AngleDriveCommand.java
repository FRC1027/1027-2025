package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveBase;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.Swerve.*;

public class AngleDriveCommand extends Command {

    private static final double TRANSLATION_TOLERANCE = 0.005;
    private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
    private static final double FIELD_WIDTH_METERS = 8.0137;

    /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
    private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
            maxSpeed * 0.5,
            maxSpeed);
    private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
            maxAngularVelocity * 0.4,
            maxAngularVelocity);

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private final SwerveBase drivetrainSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private final boolean useAllianceColor = false;

    // get values from Chris
    private double beta;
    private double distance;
    private Pose2d goalPose;

    public AngleDriveCommand(
        SwerveBase drivetrainSubsystem,
        Supplier<Pose2d> poseProvider
    ) 
    {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseProvider = poseProvider;

        xController = new ProfiledPIDController(X_kP, X_kI, X_kD, DEFAULT_XY_CONSTRAINTS);
        yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD, DEFAULT_OMEGA_CONSTRAINTS);

        xController.setTolerance(TRANSLATION_TOLERANCE);
        yController.setTolerance(TRANSLATION_TOLERANCE);
        thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, DEFAULT_OMEGA_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(THETA_TOLERANCE);

        // get angle value from Chris
        this.beta = Math.PI / 4; // sample: 45 degree
        this.distance = 1; // sample: 45 degree
        this.goalPose = CalculateGoalPose(drivetrainSubsystem.getPose(), this.beta, this.distance);

        addRequirements(drivetrainSubsystem);
    }

    public static Pose2d CalculateGoalPose(
        Pose2d initialPose,
        double beta,
        double distiance
    ) 
    {        
        System.out.print(beta);
        System.out.print(distiance);
        System.out.print(new Translation2d(Math.sin(beta), Math.cos(beta)));
        Translation2d trans = (new Translation2d(Math.sin(beta), Math.cos(beta)))
            .plus(initialPose.getTranslation())
        ;
        Rotation2d rot = initialPose.getRotation();
        
        return new Pose2d(trans, rot);        
    }

    @Override
    public void initialize() {
        resetPIDControllers();
        var pose = goalPose;
        var isRed = false;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                isRed = true;
            }
        }
        if (useAllianceColor && isRed) {
            Translation2d transformedTranslation = new Translation2d(pose.getX(), FIELD_WIDTH_METERS - pose.getY());

            Rotation2d transformedHeading = pose.getRotation().times(-1);
            pose = new Pose2d(transformedTranslation, transformedHeading);
        }
        changeWheelDirection(beta);
        xController.setGoal(pose.getX());
        yController.setGoal(pose.getY());  
    }

    private void resetPIDControllers() {
        var robotPose = poseProvider.get();
        thetaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal();
    }

    private void changeWheelDirection(double angle)
    {
        this.drivetrainSubsystem.drive(
            new Translation2d(0, 0), 
            angle, false, true);
    }
}
