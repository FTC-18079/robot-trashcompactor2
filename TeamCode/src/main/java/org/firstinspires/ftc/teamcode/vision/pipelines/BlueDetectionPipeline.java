package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.firstinspires.ftc.teamcode.util.vision.Pipeline;
import org.opencv.core.Mat;

public class BlueDetectionPipeline extends Pipeline {
    private volatile Randomization position = Randomization.RIGHT;

    // TODO: Make a real pipeline lmao
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

    public Randomization getPosition() {
        return position;
    }
}