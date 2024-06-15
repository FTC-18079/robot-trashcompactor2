package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.vision.VisionStream;

import java.lang.Math;

@TeleOp(name = "TeleOp", group = "OpModes")
public class TeleOpMode extends LinearOpMode {
    Canvas c;
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() {
        c = packet.fieldOverlay();
        Pose2d initialPose = new Pose2d(60, 12, Math.toRadians(180)); // Global.field.getRobotPose();

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

        c.setStrokeWidth(1);
        c.setStroke("#4CAF50");
        // Run until end or stopped
        while(opModeIsActive() || !isStopRequested()) {
            Drawing.drawRobot(c, robot.chassis.getPoseEstimate());
            robot.run();
        }

        // Store pose
        Global.field.setRobotPose(robot.chassis.getPoseEstimate());
        robot.reset();
    }
}
