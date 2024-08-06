package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.vision.Pipeline;
import org.firstinspires.ftc.teamcode.vision.VisionConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Config
public class PropPipeline extends Pipeline {
    private volatile Randomization position = Randomization.RIGHT;

    private final Scalar lowHSV;
    private final Scalar highHSV;
    private final Scalar color;

    // Mats we need
    private Mat erosionKernel = new Mat();
    private Mat firstCrop = new Mat();  // input cropped to make filtering easier
    private Mat bgrMat = new Mat();     // cropped input converted in BGR since RGBA can't directly convert to HSV
    private Mat hsvMat = new Mat();     // BGR mat in HSV format
    private Mat binaryMat = new Mat();  // filters out everything to black and white
    private Mat erodedMat = new Mat();  // removes small pixels to further filter
    private Mat cropped = new Mat();    // submat of eroded from which pixels are measured
    private Mat output = new Mat();     // creates boxes and labels for the image

    double maxVal = 0.0;
    int maxIndex = -1;
    public static double object = 0.002;
    Scalar sumValue = new Scalar(0);
    float totalPixels = 0.0f;
    float[] sumValNorm = new float[2];

    public PropPipeline() {
        if (Global.alliance == Global.Alliance.BLUE) {
            lowHSV = VisionConstants.lowBlueHSV;
            highHSV = VisionConstants.highBlueHSV;
            color = new Scalar(0, 0, 255);
        } else {
            lowHSV = VisionConstants.lowRedHSV;
            highHSV = VisionConstants.highRedHSV;
            color = new Scalar(255, 0, 0);
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        // Crop input to make filtering easier
        input.submat(
                new Range((int) (0.33 * input.rows()), input.rows()),
                new Range(0, input.cols())
        ).copyTo(firstCrop);

        // Convert mat to BGR then to HSV
        Imgproc.cvtColor(firstCrop, bgrMat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(bgrMat, hsvMat, Imgproc.COLOR_BGR2HSV);

        // Filter out everything not in range
        Core.inRange(hsvMat, lowHSV, highHSV, binaryMat);

        // Erode out small pixels
        Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(8, 7), new Point(-1, -1)).copyTo(erosionKernel);
        Imgproc.erode(binaryMat, erodedMat, erosionKernel);
        Imgproc.dilate(erodedMat, erodedMat, erosionKernel);

        // Copy crop to the labeled output frame
        firstCrop.copyTo(output);

        maxVal = 0.0;
        maxIndex = -1;
        float maxVal = -1.0f;
        float objectAmount = (float) object;

        // Loop through the left and right half of the frame
        for (int i = 0; i < 2; i++) {
            Imgproc.rectangle(
                    output,
                    new Point((double) (i * firstCrop.cols()) / 2, 0),
                    new Point((double) ((i + 1) * firstCrop.cols()) / 2, firstCrop.rows()),
                    color, 4
            );

            erodedMat.submat(
                    new Range(0, firstCrop.rows()),
                    new Range(i * firstCrop.cols() / 2, (i + 1) * firstCrop.cols() / 2)
            ).copyTo(cropped);

            sumValue = Core.sumElems(cropped);
            totalPixels = (float) (cropped.cols() * cropped.rows());
            sumValNorm[i] = (float) (sumValue.val[0]) / totalPixels / 255.0f;

            // Determine if prop is on screen
            if (sumValNorm[i] > maxVal) {
                maxVal = sumValNorm[i];
                maxIndex = i;
            }
        }

        if (maxVal < objectAmount) {
            maxIndex = 2;
        }

        // Set randomization
        if (maxIndex == 0) position = Randomization.LEFT;
        else if (maxIndex == 1) position = Randomization.CENTER;
        else position = Randomization.RIGHT;

        // Add number label to output
        Imgproc.putText(output, Integer.toString(maxIndex), new Point(50, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3, color, 22);

        return output;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public Randomization getPosition() {
        return position;
    }
}
