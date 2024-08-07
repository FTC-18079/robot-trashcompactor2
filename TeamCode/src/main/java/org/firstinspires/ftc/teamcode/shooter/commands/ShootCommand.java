package org.firstinspires.ftc.teamcode.shooter.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.shooter.Shooter;

public class ShootCommand extends SequentialCommandGroup {
    private final Shooter shooter;
    public ShootCommand(Shooter shooter) {
        this.shooter = shooter;

        addCommands(
                new ConditionalCommand(
                        // If in shooting mode
                        new SequentialCommandGroup(

                        ),
                        // If not in shooting mode, do nothing
                        new InstantCommand(),
                        shooter::isInShootingMode
                )
        );
        addRequirements(shooter);
    }
}
