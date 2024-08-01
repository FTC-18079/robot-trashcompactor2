package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.util.opmode.AutoPath;

public class AutoRedForward implements AutoPath {
    Chassis chassis;
    public AutoRedForward(Chassis chassis) {
        this.chassis = chassis;
    }

    @Override
    public Command generate() {
        return new WaitCommand(0);
    }
}
