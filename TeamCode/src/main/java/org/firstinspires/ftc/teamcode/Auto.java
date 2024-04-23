package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Global;

import java.lang.Math;

@Autonomous(name = "Test Autonomous", group = "Autos")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean lastUp = false;
        boolean lastDown = false;
        long delayMs = 0;

        // Run pre-auto configs
        while(opModeInInit() && !gamepad1.options) {
            // Add or remove delays
            if (lastUp != gamepad1.dpad_up && gamepad1.dpad_up) delayMs += 100;
            if (lastDown != gamepad1.dpad_down && gamepad1.dpad_down && delayMs > 0) delayMs -= 100;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            // Select alliance
            if (gamepad1.left_bumper) Global.alliance = Global.Alliance.RED;
            if (gamepad1.right_bumper) Global.alliance = Global.Alliance.BLUE;

            // Print telemetry
            telemetry.addData("Status", "Configuring Autonomous");
            telemetry.addData("Controls", "Delay: Up & Down.    Alliance: Left & Right Bumper.");
            telemetry.addData("Selected Alliance", Global.alliance == Global.Alliance.RED ? "Red" : "Blue");
            telemetry.addData("Auto delay", delayMs);
            telemetry.addLine("Press options to finish");
            telemetry.update();
        }

        telemetry.addData("Status", "Configured");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new WaitCommand(delayMs));

        // Init robot
        Pose2d initialPose = new Pose2d(12.0, -61.34, Math.toRadians(90));
        RobotCore robot = new RobotCore(
                RobotCore.OpModeType.AUTO,
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                initialPose
        );

        waitForStart();

        // TODO: replace this with object detection
        Global.randomization = 1;
        robot.scheduleAuto();
        // Close object detection portal

        // Run until end or stopped
        while(opModeIsActive() && !isStopRequested()) {
            robot.run();
            robot.updateTelemetry();
        }

        // Store pose
        Global.robotPose = robot.chassis.getPoseEstimate();
        robot.reset();
    }
}
