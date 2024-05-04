package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class VisionConstants {
    // Kalman coefficients
    public static double Q = 0.1;
    public static double R = 0.4;

    // AprilTag camera position
    public static Vector2d arducamPose = new Vector2d(3, 3); // TODO: use not made-up numbers :skull:

    // Arducam lens intrinsics
    public static final double arducam_fx = 547.622;
    public static final double arducam_fy = 547.622;
    public static final double arducam_cx = 324.306;
    public static final double arducam_cy = 243.746;

    // Object detection
    public static final String BLUE_MODEL_ASSET = "blueObject_v2.tflite";
    public static final String RED_MODEL_ASSET = "redObject_v2.tflite";

    public static final String[] BLUE_LABELS = {
            "blueObject"
    };

    public static final String[] RED_LABELS = {
            "redObject"
    };

    public static double ASPECT_RATIO = 16.0 / 9.0;
    public static float MIN_RESULT_CONFIDENCE = 0.85f;

    public static double ZOOM = 1.0;
    public static float LEFT_POS = 275f;

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
