package org.firstinspires.ftc.teamcode.chassis.drivetrains;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;

import org.firstinspires.ftc.teamcode.chassis.drivetrains.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.util.KalmanFilter;
import org.firstinspires.ftc.teamcode.vision.ATVision;
import org.firstinspires.ftc.teamcode.vision.VisionConstants;

/**
 * This class serves as an extension of Roadrunner's {@link MecanumDrive} class with {@link ATVision} localization.
 * <p>
 * Taken from https://github.com/jdhs-ftc/apriltag-quickstart, modified to our use
 */
public class ATDrive extends MecanumDrive {
    public ATVision aprilTag;
    private Pose2d localizerPose;
    private Vector2d filteredVector;
    private final KalmanFilter.Vector2dKalmanFilter filter;

    /**
     *
     * @param initialPose the starting position of the robot on the field
     * @param aprilTag the {@link ATVision} instance to get tags from
     */
    public ATDrive(Pose2d initialPose, ATVision aprilTag) {
        super(initialPose);
        this.aprilTag = aprilTag;

        // Position filter
        filter = new KalmanFilter.Vector2dKalmanFilter(VisionConstants.Q, VisionConstants.R);
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
     * Updates position estimate with the visible tags by (not) passing through a Kalman filter
     * @return the robot's velocity
     */
    @Override
    // Method for updating pose estimate with AprilTags
    public PoseVelocity2d updatePoseEstimate() {
        // RR standard: get the latest pose from the upstream updatePoseEstimate
        // that will change the pose variable to the pose based on odo or drive encoders (or OTOS)
        PoseVelocity2d posVel = super.updatePoseEstimate();
        localizerPose = pose;
        // Get the absolute position from the camera
        Vector2d aprilVector = aprilTag.getVectorBasedOnTags(localizerPose.heading.log());

        if (aprilVector != null) {
            // Don't use Kalman filter for now
            // filteredVector = filter.update(twist.value(), aprilVector);

            // Use tag pose with odometry heading since AT headings are inaccurate
            pose = new Pose2d(aprilVector, localizerPose.heading);
        } else {
            // Accept current pose if no tags are in sight
            pose = localizerPose;
        }

        FlightRecorder.write("APRILTAG_POSE", new PoseMessage(pose));
        // Use localizer speeds
        return posVel;
    }
}
