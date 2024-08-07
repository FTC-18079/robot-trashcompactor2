package org.firstinspires.ftc.teamcode.shooter.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.shooter.Shooter;

public class ShootingModeToggleCommand extends SequentialCommandGroup {
    private final Shooter shooter;
    public ShootingModeToggleCommand(Shooter shooter) {
        this.shooter = shooter;

        addCommands(
                new ConditionalCommand(
                        // Disable shooting mode
                        new SequentialCommandGroup(
                                new InstantCommand(shooter::stopShooter),   // stop the shooter
                                new InstantCommand(shooter::openSeal),      // open ring seal
                                new InstantCommand(shooter::pivotDown),     // bring pivot down
                                new WaitUntilCommand(shooter::pivotReady),  // wait until pivot is at zero
                                new InstantCommand(shooter::plateRetract)   // retract plate
                        ),
                        // Enable shooting
                        new SequentialCommandGroup(
                                new InstantCommand(shooter::readyShooter),  // spin up shooter
                                new InstantCommand(shooter::closeSeal),     // close ring seal
                                new InstantCommand(shooter::plateOut),      // bring plate out
                                new WaitCommand(300)                  // wait until plate is out
                        ),
                        shooter::isInShootingMode
                ),
                new InstantCommand(shooter::toggleShootingMode)
        );
        addRequirements(shooter);
    }
}
