package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectionPipeline extends OpenCvPipeline {
    public enum Position {
        LEFT,
        CENTER,
        RIGHT
    }

    private volatile Position position = Position.LEFT;

    // TODO: Make a real pipeline lmao
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

    public Position getPosition() {
        return position;
    }
}
