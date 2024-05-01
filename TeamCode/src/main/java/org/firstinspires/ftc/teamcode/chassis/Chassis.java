package org.firstinspires.ftc.teamcode.chassis;

import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.chassis.kinematics.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Global;

public class Chassis extends SubsystemBase {
    HardwareMap hMap;
    Telemetry telemetry;
    MecanumDrive drive;
    public Chassis(RobotCore robot, Pose2d initialPose) {
        this.hMap = robot.getHardwareMap();
        this.telemetry = robot.getTelemetry();
        // TODO: change to use ATDrive after tuning MecanumDrive
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

    @Override
    public void periodic() {
        Global.field.setRobotPose(getPoseEstimate());
    }
}
