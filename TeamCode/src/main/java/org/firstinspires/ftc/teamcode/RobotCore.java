package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.util.ActionCommand;
import org.firstinspires.ftc.teamcode.util.Field2d;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.vision.ATVision;
import org.firstinspires.ftc.vision.VisionPortal;

import java.lang.Math;

public class RobotCore extends Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    GamepadEx driveController;
    GamepadEx manipController;
    Pose2d initialPose;
    SequentialCommandGroup autoSchedule;
    ATVision atVision;
    // Subsystems
    Chassis chassis;
    // Commands
    TeleOpDriveCommand driveCommand;
    // Paths
    Action toStage1;
    Action toStage2;
    Action toStage3;
    Action toPark;
    // Drive
    private static final double DRIVE_SENSITIVITY = 1.1;
    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double DEADZONE = 0.09;

    // OpMode type enumerator
    public enum OpModeType {
        TELEOP, AUTO
    }

    public RobotCore(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamePad1, Gamepad gamePad2, Pose2d initialPose) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        atVision = new ATVision(hardwareMap);
        while (atVision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Status", "Initializing AprilTags");
            telemetry.update();
        }

        this.hardwareMap = hardwareMap;
        this.driveController = new GamepadEx(gamePad1);
        this.manipController = new GamepadEx(gamePad2);

        this.initialPose = initialPose;
        Global.field.setRobotPose(initialPose);

        // Initialize subsystems
        initSubsystems();

        // Set up OpMode
        setupOpMode(type);
        this.telemetry.addData("Status", "Robot Initialized, ready to enable");
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
                .splineToSplineHeading(new Pose2d(36, -12, 0), Math.PI / 2.0)
                .build();

        toStage2 = chassis.actionBuilder(chassis.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(36, 0, 0), Math.PI / 2.0)
                .build();

        toStage3 = chassis.actionBuilder(chassis.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(36, 12, 0), Math.PI / 2.0)
                .build();

        toPark = chassis.actionBuilder(chassis.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(40, 24, Math.toRadians(180)), Math.PI / 2.0)
                .build();
    }

    public void scheduleAuto() {
        Action toStage;
        if (Global.randomization == Global.Randomization.LEFT) toStage = toStage1;
        else if (Global.randomization == Global.Randomization.CENTER) toStage = toStage2;
        else toStage = toStage3;

        autoSchedule = new SequentialCommandGroup(
                new ActionCommand(toStage, chassis),
                new ActionCommand(toPark, chassis)
        );
        CommandScheduler.getInstance().schedule(autoSchedule);
    }

    public HardwareMap getHardwareMap() {
        return this.hardwareMap;
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
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        this.telemetry.addData("AprilTag FPS", atVision.getFPS());
        this.telemetry.update();
    }
}
