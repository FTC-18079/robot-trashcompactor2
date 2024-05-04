package org.firstinspires.ftc.teamcode.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.chassis.Chassis;

import java.util.function.DoubleSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final Chassis chassis;
    private DoubleSupplier x, y, rot;
    public TeleOpDriveCommand(Chassis chassis, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        this.chassis = chassis;
        this.x = x;
        this.y = y;
        this.rot = rot;

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        chassis.setDrivePowers(y.getAsDouble(), x.getAsDouble(), rot.getAsDouble());
    }
}
