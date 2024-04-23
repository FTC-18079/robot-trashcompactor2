package org.firstinspires.ftc.teamcode.chassis;

import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.chassis.kinematics.MecanumDrive;

public class Chassis extends SubsystemBase {
    HardwareMap hMap;
    Telemetry telemetry;
    MecanumDrive drive;
    public Chassis(HardwareMap hMap, Telemetry telemetry, Pose2d initialPose) {
        this.hMap = hMap;
        this.telemetry = telemetry;
        drive = new MecanumDrive(hMap, initialPose);
    }

    public void setPoseEstimate() {

    }

    public Pose2d getPoseEstimate() {
        return drive.pose;
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d initialPose) {
        return drive.actionBuilder(initialPose);
    }

}
