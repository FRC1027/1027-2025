package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TestCommand extends Command
{
    public TestCommand()
    {
        System.out.println("It worked!");
        //DriverStation.reportError(getName(), null);
    }
}