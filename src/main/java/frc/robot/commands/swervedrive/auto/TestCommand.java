package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class TestCommand extends Command
{
    public TestCommand()
    {

        //System.out.println("It worked!");
        //DriverStation.reportError(getName(), null);
        new PrintCommand("Maybe?");
    }
}