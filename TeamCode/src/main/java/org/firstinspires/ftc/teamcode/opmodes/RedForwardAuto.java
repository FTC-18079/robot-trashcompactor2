package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.vision.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
@Autonomous(name = "Red Forward Auto", group = "Red Autos")
public class RedForwardAuto extends LinearOpMode {
    public static double startingX = 12;
    public static double startingY = -60;
    public static double startingH = Math.toRadians(90);

    OpenCvCamera objCamera;
    DetectionPipeline pipeline;
    DetectionPipeline.Position randomization;
    boolean cameraFailed = false;
    int errorValue = 0;

    boolean liveView = false;
    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastX = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Run pre-auto configs
        Global.delayMs = 0;
        Global.alliance = Global.Alliance.RED;
        while(opModeInInit() && !gamepad1.options) {
            config();
        }

        // Init vision
        telemetry.addData("Status", "Configuring object detection");
        telemetry.update();

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

        // Init robot
        Pose2d initialPose = new Pose2d(startingX, startingY, startingH);
        RobotCore robot = new RobotCore(
                RobotCore.OpModeType.RED_FORWARD,
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
                telemetry.addData("WARNING", "Camera did not init properly. See error.");
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

        // Run opmode
        while(opModeIsActive() && !isStopRequested()) {
            robot.run();
        }

        // Store pose
        Global.field.setRobotPose(robot.getPoseEstimate());
        robot.reset();
    }

    private void config() {
        // Add or remove delays
        if (lastUp != gamepad1.dpad_up && gamepad1.dpad_up) Global.delayMs += 100;
        if (lastDown != gamepad1.dpad_down && gamepad1.dpad_down && Global.delayMs > 0) Global.delayMs -= 100;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // Toggle LiveView
        if (lastX != gamepad1.x && gamepad1.x) liveView = !liveView;

        // Print telemetry
        telemetry.addData("Status", "Configuring Autonomous");
        telemetry.addData("Controls", "Delay: Up & Down.\nToggle Live View: X/Square");
        telemetry.addLine();
        telemetry.addData("Auto delay", Global.delayMs);
        telemetry.addData("LiveView", liveView ? "Enabled" : "Disabled");
        telemetry.addLine("Press options to finish");
        telemetry.update();
    }
}
