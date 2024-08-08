package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.vision.PipelineIF;

@Config
@Autonomous(name = "Red Forward Auto", group = "Red Autos")
public class RedForwardAuto extends LinearOpMode {
    public static double startingX = 12;
    public static double startingY = -60;
    public static double startingH = Math.toRadians(90);

    PipelineIF.Randomization randomization;
    boolean cameraFailed = false;
    int errorValue = 0;
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
            telemetry.addData("Randomization", robot.getRandomization());
            if (cameraFailed) {
                telemetry.addData("WARNING", "Camera did not init properly. See error.");
                telemetry.addData("Error code", errorValue);
            }
            telemetry.update();

            // Don't kill CPU lol
            sleep(50);
        }

        randomization = robot.getRandomization();
        Global.randomization = randomization;

        // Close camera to save our CPU
        robot.closeObjectDetection();

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
        if (lastX != gamepad1.x && gamepad1.x) Global.liveView = !Global.liveView;

        // Print telemetry
        telemetry.addData("Status", "Configuring Autonomous");
        telemetry.addData("Controls", "Delay: Up & Down.\nToggle Live View: X/Square");
        telemetry.addLine();
        telemetry.addData("Auto delay", Global.delayMs);
        telemetry.addData("LiveView", Global.liveView ? "Enabled" : "Disabled");
        telemetry.addLine("Press options to finish");
        telemetry.update();
    }
}
