package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class ATVision {
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    public AprilTagDetection lastDetection;
    public ATVision(HardwareMap hMap) {
        // Create AT Processor
        tagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(VisionConstants.arducam_fx, VisionConstants.arducam_fy, VisionConstants.arducam_cx, VisionConstants.arducam_cy)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        // Create Vision Portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hMap.get(WebcamName.class, RobotMap.CAMERA_AT))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(tagProcessor)
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
                robotPos = getRobotPosition(detection, robotHeading, VisionConstants.arducamPose);

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

    public Vector2d toVector2d(VectorF vector) {
        return new Vector2d(vector.get(0), vector.get(1));
    }

    public double quarternionToHeading(Quaternion q) {
        return Math.atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x)) - Math.toRadians(270);
    }

    public Vector2d getRobotPosition(AprilTagDetection detection, double robotHeading, Vector2d cameraOffset) {
        // Calculate robot's coordinates
        double x = detection.ftcPose.x - cameraOffset.x;
        double y = detection.ftcPose.y - cameraOffset.y;
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

    // Position library credit to Michael from team 14343 (@overkil on Discord)
    public static AprilTagLibrary getCenterStageTagLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }
}
