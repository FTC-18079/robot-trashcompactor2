package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.getCenterStageTagLibrary;

import android.util.Size;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class ATVision {
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    public AprilTagDetection lastDetection;
    public final VisionStream stream = new VisionStream();

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

    // Taken from https://github.com/jdhs-ftc/apriltag-quickstart

    /**
     * Gets AprilTag detections calculates positions based on them
     * @param robotHeading the robot's current heading
     * @return a {@link Vector2d} representing the average of all AprilTag positions
     */
    public Vector2d getVectorBasedOnTags(double robotHeading) {
        return getDetections().stream()
                .map(detection -> getFCPosition(detection, robotHeading, VisionConstants.arducamPose))
                .reduce(new Vector2d(0, 0), Vector2d::plus)
                .div(getDetections().size());
    }

    public Vector2d getFCPosition(AprilTagDetection detection, double robotHeading, Vector2d cameraOffset) {
        // get coordinates of the robot in RC coordinates
        // ensure offsets are RC
        double x = detection.ftcPose.x-cameraOffset.x;
        double y = detection.ftcPose.y-cameraOffset.y;

        // invert heading to correct properly
        robotHeading = -robotHeading;

        // rotate RC coordinates to be field-centric
        double x2 = x*Math.cos(robotHeading)+y*Math.sin(robotHeading);
        double y2 = x*-Math.sin(robotHeading)+y*Math.cos(robotHeading);
        // add FC coordinates to apriltag position
        // tags is just the CS apriltag library
        VectorF tagPose = getCenterStageTagLibrary().lookupTag(detection.id).fieldPosition;

        if (!detection.metadata.name.contains("Audience")) { // is it a backdrop tag?
            return new Vector2d(
                    tagPose.get(0) + y2,
                    tagPose.get(1) - x2);

        } else {
            return new Vector2d(
                    tagPose.get(0) - y2,
                    tagPose.get(1) + x2);

        }
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
}
