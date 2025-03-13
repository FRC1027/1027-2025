package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;

public class TestCommand extends Command
{
    // public static Command TestCommandAuto()        
    // {
    //     /*
    //     //System.out.println("It worked!");
    //     DriverStation.reportError("Maybe?", true);
    //     //new PrintCommand("Maybe?");
    //     try{
    //         // Load the path you want to follow using its name in the GUI
    //         PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");
    
    //         // Create a path following command using AutoBuilder. This will also trigger event markers.
    //         return AutoBuilder.followPath(path);
    //     } catch (Exception e) {
    //         DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    //         return Commands.none();
    //     } finally {
    //         DriverStation.reportError("It Worked!", true);
    //         //new PrintCommand("It worked!");
    //     }
    //     */

    //      try{
    //     PathPlannerPath path = PathPlannerPath.fromPathFile("Test Name");

    //     return new FollowPathCommand(
    //             path,
    //             this::getPose, // Robot pose supplier
    //             this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //             this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
    //             new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                     new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                     new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //             ),
    //             Constants.robotConfig, // The robot configuration
    //             () -> {
    //               // Boolean supplier that controls when the path will be mirrored for the red alliance
    //               // This will flip the path being followed to the red side of the field.
    //               // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //               var alliance = DriverStation.getAlliance();
    //               if (alliance.isPresent()) {
    //                 return alliance.get() == DriverStation.Alliance.Red;
    //               }
    //               return false;
    //             },
    //             this // Reference to this subsystem to set requirements
    //     );
    //     } catch (Exception e) {
    //         DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    //         return Commands.none();
    //     }
    // }
}