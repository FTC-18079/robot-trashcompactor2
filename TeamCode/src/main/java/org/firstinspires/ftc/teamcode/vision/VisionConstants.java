package org.firstinspires.ftc.teamcode.vision;

public class VisionConstants {
    // Arducam lens intrinsics
    public final double arducam_fx = 547.622;
    public final double arducam_fy = 547.622;
    public final double arducam_cx = 324.306;
    public final double arducam_cy = 243.746;

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
}
