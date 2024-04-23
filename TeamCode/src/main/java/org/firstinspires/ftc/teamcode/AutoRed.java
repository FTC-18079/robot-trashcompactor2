package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Global;

import java.lang.Math;

@Autonomous(name = "Red Auto", group = "Autos")
public class AutoRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(12.0, -61.34, Math.toRadians(90));

        // Init robot
        RobotCore robot = new RobotCore(
                RobotCore.OpModeType.RED_AUTO,
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
        Global.robotPose = robot.chassis.getPoseEstimate();
        robot.reset();
    }
}
