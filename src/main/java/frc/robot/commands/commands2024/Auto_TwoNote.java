package frc.robot.commands.commands2024;

import static frc.robot.Constants2024.Swerve.drivePower;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants2024.Swerve;
import frc.robot.subsystems.subsystems2024.shooter.ShooterArm;
import frc.robot.subsystems.subsystems2024.swerve.SwerveBase;

public class Auto_TwoNote extends SequentialCommandGroup {
    private boolean AllianceColor = false;
    private SwerveBase swerve;

    public Auto_TwoNote(SwerveBase swerve){
        this.swerve = swerve;
        addCommands(
            new ShooterArm(39).shootCommand(),
            new WaitCommand(0.5),
            new PrintCommand("ARM IS SHOT AND NOW TO WAIT FOR DRIVE"),
            
            new DriveToPoseCommand(
                swerve, 
                swerve::getPose, 
                new Pose2d(-2, 0, Rotation2d.fromDegrees(180)), 
                AllianceColor),
            new PrintCommand("DRIVE TO POSE IS COMPLETE, ONTO THE NEXT THING"),
            new FetalPositionCommand(),
            new PrintCommand("FETAL POSITION COMMAND IS COMPLETE")
            //new WaitCommand(3)
            /*
            new InstantCommand(() -> RobotContainer.intakeobj.Intake()),
            new DriveToPoseCommand(
                swerve, 
                swerve::getPose, 
                new Pose2d(2.5,0,Rotation2d.fromDegrees(0)), 
                AllianceColor),
            new InstantCommand(() -> RobotContainer.intakeobj.stop()),
            new DriveToPoseCommand(
                swerve, 
                swerve::getPose, 
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                AllianceColor)
            */
            //Commands.runOnce(() -> RobotContainer.autoMoveCommand, RobotContainer.s_Swerve)
            //movement
        );
    }
}
