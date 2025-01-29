package frc.robot.commands.commands2024;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.subsystems2024.shooter.ShooterArm;

public class Auto_ShootOnly extends SequentialCommandGroup {
    public Auto_ShootOnly(){
        addCommands(
            new ShooterArm(39).shootCommand()
        );

    }
}
