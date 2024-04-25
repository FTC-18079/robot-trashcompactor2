package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.util.ActionCommand;
import org.firstinspires.ftc.teamcode.util.Global;

import java.lang.Math;

public class RobotCore extends Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    GamepadEx driveController;
    GamepadEx manipController;
    Pose2d initialPose;
    SequentialCommandGroup autoSchedule;
    // Subsystems
    Chassis chassis;
    // Paths
    Action toStage1;
    Action toStage2;
    Action toStage3;
    Action toPark;

    // OpMode type enumerator
    public enum OpModeType {
        TELEOP, AUTO
    }

    public RobotCore(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamePad1, Gamepad gamePad2, Pose2d initialPose) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.telemetry.addData("Status", "Initializing Robot");
        this.telemetry.update();

        Global.robotPose = initialPose;
        this.initialPose = initialPose;
        this.hardwareMap = hardwareMap;
        this.driveController = new GamepadEx(gamePad1);
        this.manipController = new GamepadEx(gamePad2);

        // Initialize subsystems
        initSubsystems();

        // Set up OpMode
        setupOpMode(type);
        this.telemetry.addData("Status", "Robot Initialized");
        this.telemetry.update();
    }

    public void initSubsystems() {
        chassis = new Chassis(hardwareMap, telemetry, initialPose);
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
        // Add teleop code here
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
        if (Global.randomization == 1) toStage = toStage1;
        else if (Global.randomization == 2) toStage = toStage2;
        else toStage = toStage3;

        autoSchedule = new SequentialCommandGroup(
                new ActionCommand(toStage, chassis),
                new ActionCommand(toPark, chassis)
        );
        CommandScheduler.getInstance().schedule(autoSchedule);
    }

    public void updateTelemetry() {
        this.telemetry.update();
    }
}
