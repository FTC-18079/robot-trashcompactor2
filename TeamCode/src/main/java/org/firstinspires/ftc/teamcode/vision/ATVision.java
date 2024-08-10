package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.getCenterStageTagLibrary;

import android.util.Size;

import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.util.vision.ATLivestream;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class ATVision {
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    public AprilTagDetection lastDetection;
    public final ATLivestream stream = new ATLivestream();

    public ATVision() {
        // Create AT Processor
        tagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(VisionConstants.arducam_fx, VisionConstants.arducam_fy, VisionConstants.arducam_cx, VisionConstants.arducam_cy)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        // Create Vision Portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(RobotMap.getInstance().CAMERA_AT)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(tagProcessor)
                .addProcessor(stream)
                .enableLiveView(false)
                .build();
    }

    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }

    public float getFPS() {
        return visionPortal.getFps();
    }

    public List<AprilTagDetection> getDetections() {
        return tagProcessor.getDetections();
    }

    // Taken from https://github.com/jdhs-ftc/2023

    /**
     * Gets AprilTag detections calculates positions based on them
     * @param robotHeading the robot's current heading
     * @return a {@link Vector2d} representing the average of all AprilTag positions
     */
    public Vector2d getVectorBasedOnTags(double robotHeading) {
        List<AprilTagDetection> currentDetections = getDetections();
        int realDetections = 0;
        Vector2d averagePose = new Vector2d(0, 0); // Starting point
        if (currentDetections.isEmpty()) return null;
        Vector2d robotPos;

        // Loop through detection list and calculate robot pose from each tag
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                Vector2d tagPos = toVector2d(detection.metadata.fieldPosition); // SDK builtin tag position
                double tagHeading = quarternionToHeading(detection.metadata.fieldOrientation); // SDK builtin tag heading

                //RobotPos = calculateRobotPosFromTag(tagPos, tagHeading,localizerPose.heading.log(), detection); // calculate the robot position from the tag position
                robotPos = calculateRobotPosition(detection, robotHeading, VisionConstants.arducamPose);

                // we're going to get the average here by adding them all up and dividingA the number of detections
                // we do this because the backdrop has 3 tags, so we get 3 positions
                // hopefully by averaging them we can get a more accurate position
                lastDetection = detection;
                averagePose = averagePose.plus(robotPos);
                realDetections++;
            }
        }

        return averagePose.div(realDetections);
    }

    /**
     *
     * @param vector the input {@link VectorF}
     * @return the vector as a {@link Vector2d}
     */
    public Vector2d toVector2d(VectorF vector) {
        return new Vector2d(vector.get(0), vector.get(1));
    }

    public double quarternionToHeading(Quaternion q) {
        return Math.atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x)) - Math.toRadians(270);
    }

    /**
     * Calculates a robot position based off an AprilTag detection
     * @param detection the AprilTag detection to estimate position off of
     * @param robotHeading the robot's current heading to use for the new position
     * @param cameraOffset the camera's offset from the robot's center
     * @return a {@link Vector2d} representing the robot's field position
     */
    public Vector2d calculateRobotPosition(AprilTagDetection detection, double robotHeading, Vector2d cameraOffset) {
        // Calculate robot's coordinates
        double x = detection.ftcPose.x - cameraOffset.getX();
        double y = detection.ftcPose.y - cameraOffset.getY();
        robotHeading = - robotHeading;

        // Rotate coordinates to be field-centric
        double x2 = x * Math.cos(robotHeading) + y * Math.sin(robotHeading);
        double y2 = x * -Math.sin(robotHeading) + y * Math.cos(robotHeading);

        // Get tag pose from library
        VectorF tagPose = getCenterStageTagLibrary().lookupTag(detection.id).fieldPosition;

        // Invert tag if on audience side
        if (detection.metadata.id <= 6) { // Backdrop side
            return new Vector2d(
                    tagPose.get(0) + y2,
                    tagPose.get(1) - x2
            );
        } else return new Vector2d( // Audience side
                tagPose.get(0) - y2,
                tagPose.get(1) + x2
        );
    }
}
