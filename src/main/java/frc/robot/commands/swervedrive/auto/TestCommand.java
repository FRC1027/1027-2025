package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class TestCommand extends Command
{
    public Command TestCommandAuto()        
    {
        //System.out.println("It worked!");
        //DriverStation.reportError(getName(), null);
        new PrintCommand("Maybe?");
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("Position1(R3)Coral1");
    
            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        } finally {
            new PrintCommand("It worked!");
        }
    }
}