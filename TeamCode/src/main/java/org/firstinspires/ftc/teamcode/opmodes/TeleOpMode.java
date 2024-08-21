package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;

import java.lang.Math;

@TeleOp(name = "TeleOp", group = "OpModes")
public class TeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose initialPose = new Pose(0, 0, 0); // Global.field.getRobotPose();

        // Init robot
        RobotCore robot = new RobotCore(
                RobotCore.OpModeType.TELEOP,
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                initialPose
        );

        while (opModeInInit()) {
            telemetry.addData("Alliance", Global.alliance);
            telemetry.addData("AprilTag FPS", robot.getAprilTag().getFPS());
            telemetry.update();
        }

        waitForStart();

        // Run until end or stopped
        while(opModeIsActive() || !isStopRequested()) {
            robot.run();

            // Draw robot
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#4CAF50");
            Drawing.drawRobotOnCanvas(packet.fieldOverlay(), robot.getPoseEstimate());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // Store pose
        Global.field.setRobotPose(robot.getPoseEstimate());
        robot.reset();
    }
}
