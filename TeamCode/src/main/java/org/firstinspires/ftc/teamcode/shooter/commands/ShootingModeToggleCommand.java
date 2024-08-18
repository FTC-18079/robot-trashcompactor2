package org.firstinspires.ftc.teamcode.shooter.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class ShootingModeToggleCommand extends SequentialCommandGroup {
    public ShootingModeToggleCommand(Shooter shooter) {
        addCommands(
                new ConditionalCommand(
                        // Disable shooting mode
                        new SequentialCommandGroup(
                                new InstantCommand(shooter::toggleShootingMode),     // bring pivot down
                                new InstantCommand(shooter::stopShooter),   // stop the shooter
                                new InstantCommand(shooter::openSeal),      // open ring seal
                                new WaitUntilCommand(shooter::pivotReady),  // wait until pivot is at zero
                                new InstantCommand(shooter::plateStow)      // retract plate
                        ),
                        // Enable shooting
                        new SequentialCommandGroup(
                                new InstantCommand(shooter::closeSeal),     // close ring seal
                                new InstantCommand(shooter::plateOut),      // bring plate out
                                new WaitCommand(500),                 // wait until plate is out
                                new InstantCommand(shooter::toggleShootingMode),
                                new WaitUntilCommand(shooter::pivotReady), // wait until pivot at pose
                                new InstantCommand(shooter::readyShooter)  // spin up shooter
                        ),
                        shooter::isInShootingMode
                )
        );
        addRequirements(shooter);
    }
}
