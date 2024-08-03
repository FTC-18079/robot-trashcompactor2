package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.vision.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.lang.Math;

@Autonomous(name = "Test Autonomous", group = "Autos")
public class Auto extends LinearOpMode {
    OpenCvCamera objCamera;
    DetectionPipeline pipeline;
    DetectionPipeline.Position randomization;
    boolean cameraFailed = false;
    int errorValue = 0;

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

        // Init vision
        objCamera = OpenCvCameraFactory.getInstance().createWebcam(RobotMap.getInstance().CAMERA_OBJECT);
        pipeline = new DetectionPipeline();
        objCamera.setPipeline(pipeline);

        objCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                objCamera.startStreaming(1280, 720);
            }

            @Override
            public void onError(int errorCode) {
                cameraFailed = true;
                errorValue = errorCode;
            }
        });

        CommandScheduler.getInstance().schedule(new WaitCommand(delayMs));

        // Init robot
        Pose2d initialPose = new Pose2d(12.0, -61.34, Math.toRadians(90));
        RobotCore robot = new RobotCore(
                RobotCore.OpModeType.TELEOP,
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                initialPose
        );

        // Get object detection and randomization
        while (opModeInInit()) {
            telemetry.addData("Randomization", pipeline.getPosition());
            if (cameraFailed) {
                telemetry.addData("WARNING", "Camera did not initialize");
                telemetry.addData("Error code", errorValue);
            }
            telemetry.update();

            // Don't kill CPU lol
            sleep(50);
        }
        randomization = pipeline.getPosition();
        Global.randomization = randomization;

        // Close camera to save our CPU
        objCamera.stopStreaming();
        objCamera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {}
        });

        // Schedule auto
//        robot.scheduleAuto();

        // Run until end or stopped
        while(opModeIsActive() && !isStopRequested()) {
            robot.run();
        }

        // Store pose
        Global.field.setRobotPose(robot.getPoseEstimate());
        robot.reset();
    }
}
