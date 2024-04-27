package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class TfodPipeline {
    public TfodProcessor tfodProcessor;
    public VisionPortal visionPortal;

    public TfodPipeline(HardwareMap hMap, String asset, String[] labels, boolean liveView) {
        // Create object detection processor
        tfodProcessor = new TfodProcessor.Builder()
                .setModelAssetName(asset)
                .setModelLabels(labels)
                .setModelAspectRatio(VisionConstants.ASPECT_RATIO)
                .build();
        tfodProcessor.setMinResultConfidence(VisionConstants.MIN_RESULT_CONFIDENCE);
        tfodProcessor.setZoom(VisionConstants.ZOOM);

        // Create vision processor
        // TODO: Figure out why enabling LiveView crashes code upon closing portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hMap.get(WebcamName.class, RobotMap.CAMERA_OBJECT))
                .setCameraResolution(new Size(640,480))
                .addProcessor(tfodProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(liveView)
                .setAutoStopLiveView(true)
                .build();
    }

    public Recognition getTfodDetection() {
        boolean targetTfodFound = false;
        Recognition tfodRecognition = null;
        List<Recognition> detectionList;

        detectionList = tfodProcessor.getRecognitions();
        if (detectionList.size() != 0) {
            targetTfodFound = true;
            tfodRecognition = detectionList.get(0);
        }

        return tfodRecognition;
    }

    public Global.Randomization getRandomization(Recognition recognition) {
        if (recognition == null) return Global.Randomization.RIGHT;
        double pos = (recognition.getRight() + recognition.getLeft()) / 2;

        if (pos < VisionConstants.LEFT_POS) {
            return Global.Randomization.LEFT;
        } else {
            return Global.Randomization.CENTER;
        }
    }
}
