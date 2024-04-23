package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotCore extends Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    GamepadEx driveController;
    GamepadEx manipController;
    Pose2d initialPose;

    // Subsystems
    // TODO: add subsystems here

    // OpMode type enumerator
    public enum OpModeType {
        TELEOP, RED_AUTO, BLUE_AUTO
    }

    public RobotCore(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamePad1, Gamepad gamePad2, Pose2d initialPose) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.telemetry.addData("Status", "Initializing");
        this.telemetry.update();

        this.hardwareMap = hardwareMap;
        this.driveController = new GamepadEx(gamePad1);
        this.manipController = new GamepadEx(gamePad2);

        this.initialPose = initialPose;

        // Init subsystems
        // TODO: init subsystems here
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

    }

    private void initRedAuto() {

    }

    private void initBlueAuto() {

    }

    public void updateTelemetry() {
        this.telemetry.update();
    }
}
