package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.opmode.AutoPath;
import org.firstinspires.ftc.teamcode.util.vision.PipelineIF;

public class BlueForwardSequence implements AutoPath {
    Chassis chassis;
    public BlueForwardSequence(Chassis chassis) {
        this.chassis = chassis;
    }

    @Override
    public Command generate() {
        PipelineIF.Randomization randomization = Global.randomization;
        return new WaitCommand(Global.delayMs)
//                .andThen(new ActionCommand(
//                        chassis.actionBuilder(chassis.getPoseEstimate())
//                                .splineToSplineHeading(new Pose2d(20, -30, Math.toRadians(180)), Math.PI / 2)
//                                .build()))
                ;
    }
}
