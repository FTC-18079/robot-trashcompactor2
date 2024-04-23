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

    // Subsystems
    Chassis chassis;

    // OpMode type enumerator
    public enum OpModeType {
        TELEOP, RED_AUTO, BLUE_AUTO
    }

    public RobotCore(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamePad1, Gamepad gamePad2, Pose2d initialPose) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.telemetry.addData("Status", "Initializing Robot");
        this.telemetry.update();

        this.hardwareMap = hardwareMap;
        this.driveController = new GamepadEx(gamePad1);
        this.manipController = new GamepadEx(gamePad2);

        // Init subsystems
        chassis = new Chassis(hardwareMap, telemetry, initialPose);
        Global.robotPose = initialPose;

        // Set up OpMode
        setupOpMode(type);
        this.telemetry.addData("Status", "Robot Initialized");
        this.telemetry.update();
    }

    private void setupOpMode(OpModeType type) {
        switch (type) {
            case TELEOP:
                initTeleOp();
                break;
            case RED_AUTO:
                initRedAuto();
                break;
            case BLUE_AUTO:
                initBlueAuto();
                break;
        }
    }

    private void initTeleOp() {
        // Add teleop code here
    }

    private void initRedAuto() {
        Action toStage1;
        Action toStage2;
        Action toStage3;

        toStage1 = chassis.actionBuilder(chassis.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(36, -12, 0), Math.PI / 2.0)
                .build();

        toStage2 = chassis.actionBuilder(chassis.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(36, 0, 0), Math.PI / 2.0)
                .build();

        toStage3 = chassis.actionBuilder(chassis.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(36, 12, 0), Math.PI / 2.0)
                .build();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
            new ActionCommand(toStage1, chassis)
        ));
    }

    private void initBlueAuto() {

    }

    public void updateTelemetry() {
        this.telemetry.update();
    }
}
