package org.firstinspires.ftc.teamcode.shooter.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.shooter.Shooter;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(Shooter shooter) {
        addCommands(
                new ConditionalCommand(
                        // If in shooting mode
                        new SequentialCommandGroup(
                            new WaitUntilCommand(shooter::isReadyToFire),
                            new InstantCommand(shooter::shoot),
                            new WaitCommand(250),
                            new InstantCommand(shooter::flickBack),
                            new WaitCommand(250)
                        ),
                        // If not in shooting mode, do nothing
                        new WaitCommand(0),
                        shooter::isInShootingMode
                )
        );
        addRequirements(shooter);
    }
}
