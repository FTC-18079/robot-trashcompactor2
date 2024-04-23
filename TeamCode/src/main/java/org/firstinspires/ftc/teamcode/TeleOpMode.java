package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "TeleOp", group = "OpModes")
public class TeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = Global.robotPose;

        // Init robot
        RobotCore robot = new RobotCore(
                RobotCore.OpModeType.TELEOP,
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                initialPose
        );

        waitForStart();

        // Run until end or stopped
        while(opModeIsActive() || !isStopRequested()) {
            robot.run();
            robot.updateTelemetry();
        }

        // Store pose
        // Global.robotPose = robot.chassis.getPoseEstimate();
        robot.reset();
    }
}
