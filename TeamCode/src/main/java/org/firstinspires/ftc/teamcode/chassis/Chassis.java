package org.firstinspires.ftc.teamcode.chassis;

import static org.firstinspires.ftc.teamcode.util.Global.Alliance.BLUE;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.chassis.kinematics.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
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
        drive = new ATDrive(hMap, initialPose, robot.atVision);
    }

    /**
     * Drives the robot
     * @param x the strafe movement input
     * @param y the forward movement input
     * @param turn the rotation movement input
     */
    public void setDrivePowers(double x, double y, double turn) {
        Vector2d input = new Vector2d(-y, -x);
        double rotationAmount = -drive.pose.heading.log();

        if (isFieldCentric) {
          if (Global.alliance == BLUE) rotationAmount = rotationAmount - Math.toRadians(90);
          else rotationAmount = rotationAmount + Math.toRadians(90);

          input = Rotation2d.fromDouble(rotationAmount).times(new Vector2d(input.x, input.y));
        }

        PoseVelocity2d velocity = new PoseVelocity2d(
                new Vector2d(
                        input.x,
                        input.y
                ), -turn
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
