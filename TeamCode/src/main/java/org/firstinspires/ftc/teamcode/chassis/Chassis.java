package org.firstinspires.ftc.teamcode.chassis;

import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.chassis.kinematics.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.util.Global;

import java.lang.Math;

public class Chassis extends SubsystemBase {
    HardwareMap hMap;
    Telemetry telemetry;
    MecanumDrive drive;
    boolean isFieldCentric;
    public Chassis(RobotCore robot, Pose2d initialPose) {
        this.hMap = robot.getHardwareMap();
        this.telemetry = robot.getTelemetry();
        isFieldCentric = true;
        // TODO: change to use ATDrive after tuning MecanumDrive
        drive = new MecanumDrive(hMap, initialPose);
    }

    /**
     * Drives the robot with heading in relation to the field
     * @param drive the forward movement input
     * @param strafe the strafe movement input
     * @param turn the rotation movement input
     */
    public void driveFieldCentric(double drive, double strafe, double turn) {
        Vector2d angleVector = this.drive.pose.heading.vec();
        double angle = -Math.atan2(angleVector.y, angleVector.x);

        double rotatedX = strafe * Math.cos(angle) - drive * Math.sin(angle);
        double rotatedY = strafe * Math.sin(angle) + drive * Math.cos(angle);

        setDrivePowers(rotatedY, rotatedX, turn);
    }

    /**
     * Drives the robot with heading in relation to itself
     * @param drive the forward movement input
     * @param strafe the strafe movement input
     * @param turn the rotation movement input
     */
    public void setDrivePowers(double drive, double strafe, double turn) {
        PoseVelocity2d velocity = new PoseVelocity2d(
                new Vector2d(
                        -drive,
                        strafe
                ), turn
        );
        this.drive.setDrivePowers(velocity);
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public Pose2d getPoseEstimate() {
        return drive.pose;
    }

    public boolean isFieldCentric() {
        return isFieldCentric;
    }

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    // Sets the robot's current heading to be the zero angle
    public void resetHeading() {
        this.drive.pose = new Pose2d(
                this.drive.pose.position,
                Rotation2d.exp(0)
        );
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d initialPose) {
        return drive.actionBuilder(initialPose);
    }

    @Override
    public void periodic() {
        updatePoseEstimate();
        telemetry.addData("Pose estimate", drive.pose.toString());
    }
}
