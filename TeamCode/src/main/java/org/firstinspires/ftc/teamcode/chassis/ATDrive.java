package org.firstinspires.ftc.teamcode.chassis;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.chassis.kinematics.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.vision.ATVision;

/**
 * This class serves as an extension of Roadrunner's {@link MecanumDrive} class with {@link ATVision} localization.
 * <p>
 * Taken from https://github.com/jdhs-ftc/2023, modified to our use
 */
public class ATDrive extends MecanumDrive {
    public ATVision aprilTag;
    private Pose2d localizerPose;
    private Vector2d filteredVector;

    /**
     *
     * @param hMap the hardwareMap instance needed to set up hardware
     * @param initialPose the starting position of the robot on the field
     * @param aprilTag the {@link ATVision} instance to get tags from
     */
    public ATDrive(HardwareMap hMap, Pose2d initialPose, ATVision aprilTag) {
        super(hMap, initialPose);
        this.aprilTag = aprilTag;
    }

    /**
     * Relocalizes robot
     * @param newPose the new position of the robot
     */
    public void setPoseEstimate(Pose2d newPose) {
        if (newPose != null) {
            pose = newPose;
            poseHistory.add(pose);

            while (poseHistory.size() > 100) {
                poseHistory.removeFirst();
            }

            estimatedPoseWriter.write(new PoseMessage(pose));
        }
    }

    /**
     * Updates position estimate with the visible tags by passing through a Kalman filter
     * @return the robot's velocity
     */
    @Override
    // Method for updating pose estimate with AprilTags
    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        localizerPose = pose.plus(twist.value());

        Vector2d aprilVector = aprilTag.getVectorBasedOnTags(localizerPose.heading.log());

        if (aprilVector != null) {
            // If tags are visible, use the apriltag position with localizer heading
            // Input the change from odometry with the april absolute pose into the kalman filter
            filteredVector = new Vector2d(0, 0); // TODO: replace this with a Kalman filter, filter.update(twist.value(), aprilVector);
            // Then we add the filtered position to the localizer heading as a pose
            // TODO: This currently uses just AT pose. aprilVector should be switched for filteredVector to use kalman filter once it's added.
            pose = new Pose2d(aprilVector, localizerPose.heading);
        } else {
            // If no tags are visible, use the localizer position to update the kalman filter
            // TODO: uncomment this once we actually have a filter :skull:
            // filteredVector = posFilter.update(twist.value(), localizerPose.position);

            // Just use existing pose (lmao)
            pose = localizerPose;
        }

        // Roadrunner stuff, directly from the overriden method (overrode according to merriam-webster)
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }
}
