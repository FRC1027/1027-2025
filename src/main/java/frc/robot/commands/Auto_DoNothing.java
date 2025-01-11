package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto_DoNothing extends SequentialCommandGroup {
    public Auto_DoNothing(){
        addCommands(
            new InstantCommand()
            //new InstantCommand(RobotContainer.autoMoveCommand)
        );
    }
}
