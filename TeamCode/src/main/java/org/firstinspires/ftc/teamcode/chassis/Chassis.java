package org.firstinspires.ftc.teamcode.chassis;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class Chassis extends SubsystemBase {
    Telemetry telemetry;
    Follower follower;
    boolean isRobotCentric;
    public Chassis(RobotCore robot, Pose initialPose) {
        this.telemetry = robot.getTelemetry();
        isRobotCentric = false;
        follower = new Follower(initialPose);
    }

    /**
     * Drives the robot
     * @param fwd the forward movement input
     * @param str the strafe movement input
     * @param rot the rotation movement input
     */
    public void setDrivePowers(double fwd, double str, double rot) {
        follower.setTeleOpMovementVectors(fwd, str, rot, isRobotCentric);
    }

    public Pose getPoseEstimate() {
        return follower.getPose();
    }

    public boolean isRobotCentric() {
        return isRobotCentric;
    }

    public void toggleRobotCentric() {
        isRobotCentric = !isRobotCentric;
    }

    // Sets the robot's current heading to be the zero angle
    public void resetHeading() {
        Pose oldPose = getPoseEstimate();
        follower.setPose(new Pose(oldPose.getX(), oldPose.getY(), 0));
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    @Override
    public void periodic() {
        follower.update();
        telemetry.addData("Robot centric", isRobotCentric);
//        telemetry.addData("Pose estimate", drive.pose.toString());
    }
}
