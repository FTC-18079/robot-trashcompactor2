package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.BlueForwardSequence;
import org.firstinspires.ftc.teamcode.auto.RedForwardSequence;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.opmode.AutoPath;
import org.firstinspires.ftc.teamcode.util.vision.PipelineIF;
import org.firstinspires.ftc.teamcode.vision.ATVision;
import org.firstinspires.ftc.teamcode.vision.ObjectDetection;

import java.lang.Math;

public class RobotCore extends Robot {
    RobotMap robotMap;
    Telemetry telemetry;
    GamepadEx driveController;
    GamepadEx manipController;
    Pose2d initialPose;
    ATVision atVision;
    ObjectDetection objectDetection;

    // Subsystems
    Chassis chassis;
    Intake intake;
    // Commands
    TeleOpDriveCommand driveCommand;
    SelectCommand intakeCommand;
    // Drive
    private static final double DRIVE_SENSITIVITY = 1.1;
    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double JOYSTICK_DEADZONE = 0.09;
    private static final double TRIGGER_DEADZONE = 0.05;
    // Loop times
    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;
    // OpMode type enumerator
    AutoPath autoPath;
    public enum OpModeType {
        TELEOP, RED_FORWARD, BLUE_FORWARD
    }

    public RobotCore(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamePad1, Gamepad gamePad2, Pose2d initialPose) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initializing Hardware");
        telemetry.update();
        robotMap = RobotMap.getInstance();
        robotMap.init(hardwareMap);

        telemetry.addData("Status", "Initializing AprilTags");
        telemetry.update();
        atVision = new ATVision();

        telemetry.addData("Status", "Initializing Object Detection");
        telemetry.update();
        objectDetection = new ObjectDetection(this, Global.liveView);

        FtcDashboard.getInstance().startCameraStream(atVision.stream, 15);

        this.driveController = new GamepadEx(gamePad1);
        this.manipController = new GamepadEx(gamePad2);

//        this.initialPose = initialPose;
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
        intake = new Intake(this);

        register(chassis);
        register(intake);
    }

    private void setupOpMode(OpModeType type) {
        switch (type) {
            case TELEOP:
                setDriveControls();
                closeObjectDetection();
                break;
            case RED_FORWARD:
                autoPath = new RedForwardSequence(chassis);
                break;
            case BLUE_FORWARD:
                autoPath = new BlueForwardSequence(chassis);
                break;

        }
        if (type != OpModeType.TELEOP) schedule(autoPath.generate());
    }

    private void setDriveControls() {
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

        // Intake controls
        new Trigger(() -> driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(intake::in)
                .whenInactive(intake::stop);
        new Trigger(() -> driveController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(intake::eject)
                .whenInactive(intake::stop);

//        driveController.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(intake::setupMotors);

        // Set default commands
        chassis.setDefaultCommand(driveCommand);
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

    public ATVision getAprilTag() {
        return atVision;
    }

    public PipelineIF.Randomization getRandomization() {
        return objectDetection.getPosition();
    }

    public void closeObjectDetection() {
        objectDetection.closeCamera();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        double loop = System.nanoTime();

        this.telemetry.addData("AprilTag FPS", atVision.getFPS());
        this.telemetry.addData("hz", 1000000000 / (loop - loopTime));
        this.telemetry.addData("Runtime", endTime == 0 ? timer.seconds() : endTime);
        loopTime = loop;

        this.telemetry.update();
    }

    public Pose2d getPoseEstimate() {
        return chassis.getPoseEstimate();
    }
}
