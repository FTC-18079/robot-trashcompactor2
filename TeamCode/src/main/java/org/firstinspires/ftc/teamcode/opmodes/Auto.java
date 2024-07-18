package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.vision.TfodPipeline;
import org.firstinspires.ftc.teamcode.vision.VisionConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.lang.Math;

@Autonomous(name = "Test Autonomous", group = "Autos")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() {
        long delayMs = 0;
        boolean liveView = false;

        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastX = false;

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

            // Toggle LiveView
            if (lastX != gamepad1.x && gamepad1.x) liveView = !liveView;

            // Print telemetry
            telemetry.addData("Status", "Configuring Autonomous");
            telemetry.addData("Controls", "Delay: Up & Down.  Alliance: Left & Right Bumper.  Toggle Live View: X/Square");
            telemetry.addData("Selected Alliance", Global.alliance == Global.Alliance.RED ? "\uD83D\uDD34 Red" : "\uD83D\uDD35 Blue");
            telemetry.addData("Auto delay", delayMs);
            telemetry.addData("LiveView", liveView ? "Enabled" : "Disabled");
            telemetry.addLine("Press options to finish");
            telemetry.update();
        }

        if (Global.alliance == null) Global.alliance = Global.Alliance.RED;
        telemetry.addData("Status", "Creating Vision Processor");
        telemetry.update();

        String asset = Global.alliance == Global.Alliance.RED ?
                VisionConstants.RED_MODEL_ASSET : VisionConstants.BLUE_MODEL_ASSET;
        String[] labels = Global.alliance == Global.Alliance.RED ?
                VisionConstants.RED_LABELS : VisionConstants.BLUE_LABELS;

        TfodPipeline tfod = new TfodPipeline(asset, labels, liveView);
        while (tfod.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Status", "Initializing Object Detection");
            telemetry.update();
        }

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

        // Get object detection and randomization
        Recognition recognition = null;
        while (opModeInInit()) {
            recognition = tfod.getTfodDetection();
            telemetry.addData("Randomization", recognition.toString());
        }
        Global.randomization = tfod.getRandomization(recognition);

        // Close vision portal
        tfod.visionPortal.close();

        // Schedule auto
        robot.scheduleAuto();

        // Run until end or stopped
        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Funsies", "cool");
            robot.run();
        }

        // Store pose
        Global.field.setRobotPose(robot.getPoseEstimate());
        robot.reset();
    }
}
