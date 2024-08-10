package org.firstinspires.ftc.teamcode.chassis;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;

import java.lang.Math;

public class Chassis extends SubsystemBase {
    Telemetry telemetry;
//    MecanumDrive drive;
    boolean isFieldCentric;
    public Chassis(RobotCore robot, Pose2d initialPose) {
        this.telemetry = robot.getTelemetry();
        isFieldCentric = true;
//        drive = new ATDrive(initialPose, robot.getAprilTag());
    }

    /**
     * Drives the robot
     * @param x the strafe movement input
     * @param y the forward movement input
     * @param turn the rotation movement input
     */
    public void setDrivePowers(double x, double y, double turn) {
//        Vector2d input = new Vector2d(x, -y);
//        double rotationAmount = -drive.pose.heading.log();
//
//        // TODO: fix whatever this is :skull:
//        if (isFieldCentric) {
//          if (Global.alliance == BLUE) rotationAmount = rotationAmount - Math.toRadians(90);
//          else if (Global.alliance == RED) rotationAmount = rotationAmount + Math.toRadians(90);
//
//          input = Rotation2d.fromDouble(rotationAmount).times(new Vector2d(input.x, input.y));
//        }
//
//        PoseVelocity2d velocity = new PoseVelocity2d(
//                new Vector2d(
//                        input.x,
//                        input.y
//                ), -turn
//        );
//        this.drive.setDrivePowers(velocity);
    }

//    public void updatePoseEstimate() {
//        drive.updatePoseEstimate();
//    }

    public Pose2d getPoseEstimate() {
        return new Pose2d();
    }

    public boolean isFieldCentric() {
        return isFieldCentric;
    }

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    // Sets the robot's current heading to be the zero angle
    public void resetHeading() {
//        this.drive.pose = new Pose2d(
//                this.drive.pose.position,
//                Rotation2d.exp(0)
//        );
    }

    @Override
    public void periodic() {
//        updatePoseEstimate();
        telemetry.addData("Field centric", isFieldCentric);
//        telemetry.addData("Pose estimate", drive.pose.toString());
    }
}
