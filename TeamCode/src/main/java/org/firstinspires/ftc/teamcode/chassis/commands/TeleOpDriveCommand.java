package org.firstinspires.ftc.teamcode.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.chassis.Chassis;

import java.util.function.DoubleSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final Chassis chassis;
    private DoubleSupplier fwd, strafe, rot;
    public TeleOpDriveCommand(Chassis chassis, DoubleSupplier fwd, DoubleSupplier strafe, DoubleSupplier rot) {
        this.chassis = chassis;
        this.fwd = fwd;
        this.strafe = strafe;
        this.rot = rot;

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        chassis.setDrivePowers(fwd.getAsDouble(), strafe.getAsDouble(), rot.getAsDouble());
    }
}
