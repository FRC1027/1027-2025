package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterArm;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.robot.util.NavXGyro;

public class Auto_OneNote extends SequentialCommandGroup {
    private boolean AllianceColor = false;

    public Auto_OneNote(SwerveBase swerve){
        addCommands(
            new ShooterArm(39).shootCommand(),
            new PrintCommand("ARM IS SHOT"),
            new WaitCommand(1),
            new DriveToPoseCommand(
                swerve, 
                swerve::getPose, 
                new Pose2d(-2, 0, Rotation2d.fromDegrees(180)), 
                AllianceColor),
            new PrintCommand("DRIVE IS DONE")
        );

    }
}
