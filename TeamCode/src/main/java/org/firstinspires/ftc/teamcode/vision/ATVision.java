package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class ATVision {
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    public ATVision(HardwareMap hMap, Telemetry telemetry) {
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
}
