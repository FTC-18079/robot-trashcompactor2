package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.graphics.Canvas;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedPipeline extends OpenCvPipeline {
//    private volatile Randomization position = Randomization.RIGHT;

    public static Scalar lowHSV = new Scalar(0, 150, 0); // lenient lower bound HSV for yellow
    public static Scalar highHSV = new Scalar(11, 255, 255); // lenient higher bound HSV for yellow

    public static Scalar strictLowHSV = new Scalar(0, 200, 0); // Strict lower bound HSV for yellow
    public static Scalar strictHighHSV = new Scalar(180, 255, 255); // Strict higher bound for HSV for yellow

    // Mats we need
    private Mat erosionKernel = new Mat();
    private Mat bgrMat = new Mat();             // input in BGR
    private Mat hsvMat = new Mat();             // BGR mat in HSV format
    private Mat binaryMat = new Mat();          // filters out everything to black and white
    private Mat erodedMat = new Mat();          // removes small pixels to further filter
    private Mat maskedInputMat = new Mat();     // colors in binary mat with input's colors
    private Mat scaledMask = new Mat();         // maskedInputMat with rescaled saturation
    private Mat scaledBinary = new Mat();       // scaledMask with stricter HSV range
    private Mat finalMask = new Mat();          // colored in scaledBinary
    private Mat finalRgb = new Mat();           // finalMask in rgb format

    @Override
    public Mat processFrame(Mat input) {
        // Convert mat into HSV
        Imgproc.cvtColor(input, bgrMat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(bgrMat, hsvMat, Imgproc.COLOR_BGR2HSV);

        // Create a black and white of everything out of color range
        Core.inRange(hsvMat, lowHSV, highHSV, binaryMat);

        // Erode and dilate image to get rid of small pixels
        erosionKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(8, 7), new Point(-1, -1));
        Imgproc.erode(binaryMat, erodedMat, erosionKernel);
        Imgproc.dilate(erodedMat, erodedMat, erosionKernel);

        // Color in binary mat
        maskedInputMat.release();
        Core.bitwise_and(hsvMat, hsvMat, maskedInputMat, erodedMat);

        // Calculate average HSV and scale average saturation to 255
        Scalar average = Core.mean(maskedInputMat, binaryMat);
        maskedInputMat.convertTo(scaledMask, -1, 255 / average.val[1], 0);

        // Apply strict HSV filter to scaledMask
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledBinary);

        // Color in scaledBinary
        finalMask.release();
        Core.bitwise_and(hsvMat, hsvMat, finalMask, scaledBinary);

        // Convert finalMask to RGB
        Imgproc.cvtColor(finalMask, finalRgb, Imgproc.COLOR_HSV2RGB);

        return finalRgb;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

//    @Override
//    public Randomization getPosition() {
//        return position;
//    }
}
