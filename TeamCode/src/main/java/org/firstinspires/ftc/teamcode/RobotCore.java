package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.util.ftclib.ActionCommand;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.vision.ATVision;
import org.firstinspires.ftc.vision.VisionPortal;

import java.lang.Math;

public class RobotCore extends Robot {
    RobotMap robotMap;
    Telemetry telemetry;
    GamepadEx driveController;
    GamepadEx manipController;
    Pose2d initialPose;
    SequentialCommandGroup autoSchedule;
    public ATVision atVision;
    // Subsystems
    Chassis chassis;
    // Commands
    TeleOpDriveCommand driveCommand;
    // Paths
    Action toStage1;
    Action toStage2;
    Action toStage3;
    Action toPark;
    Action end;
    // Drive
    private static final double DRIVE_SENSITIVITY = 1.1;
    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double DEADZONE = 0.09;
    // Loop times
    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;
    // OpMode type enumerator
    public enum OpModeType {
        TELEOP, AUTO
    }

    public RobotCore(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamePad1, Gamepad gamePad2, Pose2d initialPose) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotMap = RobotMap.getInstance();
        robotMap.init(hardwareMap);

        atVision = new ATVision();
        FtcDashboard.getInstance().startCameraStream(atVision.stream, 15);
        while (atVision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            this.telemetry.addData("Status", "Initializing AprilTags");
            this.telemetry.update();
        }

        this.driveController = new GamepadEx(gamePad1);
        this.manipController = new GamepadEx(gamePad2);

        this.initialPose = initialPose;
        Global.field.setRobotPose(initialPose);

        // Initialize subsystems
        initSubsystems();

        // Set up OpMode
        setupOpMode(type);
        this.telemetry.addData("Status", "Robot Initialized, ready to enable");
        this.telemetry.addData("Alliance", Global.alliance);
        this.telemetry.update();
    }

    public void initSubsystems() {
        this.telemetry.addData("Status", "Initializing Subsystems");
        this.telemetry.update();
        chassis = new Chassis(this, initialPose);

        register(chassis);
    }

    private void setupOpMode(OpModeType type) {
        switch (type) {
            case TELEOP:
                initTeleOp();
                break;
            case AUTO:
                initAuto();
                break;
        }
    }

    private void initTeleOp() {
        // Drive command
        driveCommand = new TeleOpDriveCommand(
                chassis,
                () -> responseCurve(driveController.getLeftX(), DRIVE_SENSITIVITY),
                () -> responseCurve(driveController.getLeftY(), DRIVE_SENSITIVITY),
                () -> responseCurve(driveController.getRightX(), ROTATIONAL_SENSITIVITY)
        );

        // Toggle field centric
        driveController.getGamepadButton(GamepadKeys.Button.X)
                        .whenPressed(chassis::toggleFieldCentric);
        // Reset robot heading
        driveController.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(chassis::resetHeading);

        // Set default commands
        chassis.setDefaultCommand(driveCommand);
    }

    private void initAuto() {
        toStage1 = chassis.actionBuilder(chassis.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(40, -24, 0), Math.PI / 2.0)
                .build();

        toStage2 = chassis.actionBuilder(chassis.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(40, -12, 0), Math.PI / 2.0)
                .build();

        toStage3 = chassis.actionBuilder(chassis.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(40, 0, 0), Math.PI / 2.0)
                .build();

        toPark = chassis.actionBuilder(chassis.getPoseEstimate())
                .strafeToSplineHeading(new Vector2d(40, -40), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60, -58, Math.toRadians(90)), Math.toRadians(0))
                .build();

        end = chassis.actionBuilder(chassis.getPoseEstimate())
                .strafeToConstantHeading(new Vector2d(35, -58))
                .splineToSplineHeading(new Pose2d(12, -12, Math.toRadians(180)), Math.toRadians(90))
                .build();
    }

    public void scheduleAuto() {
        Action toStage;
        if (Global.randomization == Global.Randomization.LEFT) toStage = toStage1;
        else if (Global.randomization == Global.Randomization.CENTER) toStage = toStage2;
        else toStage = toStage3;

        autoSchedule = new SequentialCommandGroup(
                new ActionCommand(toStage, chassis),
                new WaitCommand(1000),
                new ActionCommand(toPark, chassis),
                new WaitCommand(1000),
                new ActionCommand(end, chassis)
        );
        CommandScheduler.getInstance().schedule(autoSchedule);
    }

    public Telemetry getTelemetry() {
        return this.telemetry;
    }

    // bear metal <3
    /**
     * Reduces the sensitivity around the zero point to make the Robot more
     * controllable.
     *
     * @param value raw input
     * @param power 1.0 indicates linear (full sensitivity) - larger number
     *              reduces small values
     * @return 0 to +/- 100%
     */
    public double responseCurve(double value, double power) {
//        value = deadzone(value, DEADZONE);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }

    public double deadzone(double value, double deadZone) {
        if (Math.abs(value) > deadZone) {
            if (value > 0.0) {
                return (value - deadZone) / (1.0 - deadZone);
            } else {
                return (value + deadZone) / (1.0 - deadZone);
            }
        } else {
            return 0.0;
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        double loop = System.nanoTime();

        this.telemetry.addData("AprilTag FPS", atVision.getFPS());
        this.telemetry.addData("hz", 1000000000 / (loop - loopTime));
        this.telemetry.addData("Runtime", endTime == 0 ? timer.seconds() : endTime);
        loopTime = loop;

//        this.telemetry.update();
    }

    public Pose2d getPoseEstimate() {
        return chassis.getPoseEstimate();
    }
}
