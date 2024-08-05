package org.firstinspires.ftc.teamcode.vision2.pipelines;

import org.firstinspires.ftc.teamcode.util.vision.Pipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RedDetectionPipeline extends Pipeline {
    private volatile Randomization position = Randomization.RIGHT;

    // TODO: Make a real pipeline lmao
    // Mats we need
    Mat hsvMat = new Mat();
    Mat binaryMat = new Mat();
    Mat masked = new Mat();
    Mat scaledMask = new Mat();
    Mat scaledThresh = new Mat();
    Mat finalMask = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Convert mat into HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        if (hsvMat.empty()) return input;

        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

        // Get a black and white image of yellow objects
        Core.inRange(hsvMat, lowHSV, highHSV, binaryMat);

        // Color white portion of thresh with HSV from mat
        // Output into masked
        Core.bitwise_and(hsvMat, hsvMat, masked, binaryMat);
        // Calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, binaryMat);

        // Scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);

        Scalar strictLowHSV = new Scalar(0, 150, 100); // Strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, 255, 255); // Strict higher bound for HSV for yellow

        // Apply strict HSV filter onto scaledMask to get rid of any yellow other than pale
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        // Color in scaledThresh with HSV
        Core.bitwise_and(hsvMat, hsvMat, finalMask, scaledThresh);

        input.release();
        scaledThresh.release();
        scaledMask.release();
        hsvMat.release();
        masked.release();
        binaryMat.release();
        finalMask.copyTo(input);
        finalMask.release();

        return input;
    }

    public Randomization getPosition() {
        return position;
    }
}
