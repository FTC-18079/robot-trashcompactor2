package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.graphics.Canvas;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePipeline extends OpenCvPipeline {//, PipelineIF {
//    private volatile Randomization position = Randomization.RIGHT;

    public static Scalar lowHSV = new Scalar(100, 160, 60); // lenient lower bound HSV for yellow
    public static Scalar highHSV = new Scalar(130, 255, 255); // lenient higher bound HSV for yellow

    // Mats we need
    private Mat hsvMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat maskedInputMat = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Convert mat into HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvMat, lowHSV, highHSV, binaryMat);

        maskedInputMat.release();

        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        return maskedInputMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

//    @Override
//    public Randomization getPosition() {
//        return position;
//    }
}
