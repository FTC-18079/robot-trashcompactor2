package org.firstinspires.ftc.teamcode.chassis;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;

public class Chassis extends SubsystemBase {
    Telemetry telemetry;
    Follower drive;
    boolean isFieldCentric;
    public Chassis(RobotCore robot, Pose initialPose) {
        this.telemetry = robot.getTelemetry();
        isFieldCentric = true;
        drive = new Follower(initialPose);
    }

    /**
     * Drives the robot
     * @param fwd the forward movement input
     * @param str the strafe movement input
     * @param rot the rotation movement input
     */
    public void setDrivePowers(double fwd, double str, double rot) {
        drive.setTeleOpMovementVectors(fwd, -str, -rot, !isFieldCentric);
    }

    public Pose getPoseEstimate() {
        return drive.getPose();
    }

    public boolean isFieldCentric() {
        return isFieldCentric;
    }

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    // Sets the robot's current heading to be the zero angle
    public void resetHeading() {
        Pose oldPose = getPoseEstimate();
        drive.setPose(new Pose(oldPose.getX(), oldPose.getY(), 0));
    }

    public void startTeleopDrive() {
        drive.startTeleopDrive();
    }

    @Override
    public void periodic() {
        drive.update();
        Drawing.drawRobot(getPoseEstimate(), "#4CAF50");
        telemetry.addData("Field centric", isFieldCentric);
//        telemetry.addData("Pose estimate", drive.pose.toString());
    }
}
