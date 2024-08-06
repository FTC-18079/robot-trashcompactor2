package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.graphics.Canvas;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePipeline extends OpenCvPipeline {//, PipelineIF {
//    private volatile Randomization position = Randomization.RIGHT;

    public static Scalar lowHSV = new Scalar(100, 160, 60); // lenient lower bound HSV for yellow
    public static Scalar highHSV = new Scalar(130, 255, 255); // lenient higher bound HSV for yellow

    // Mats we need
    private Mat mat = new Mat();
    private Mat erosionKernel = new Mat();
    private Mat boxedInput = new Mat();
    private Mat cropped = new Mat();

    double maxVal = 0.0;
    int maxIndex = -1;
    public static double object = 0.001;
    Scalar sumValue = new Scalar(0);
    float totalPixels = 0.0f;
    float[] sumValNorm = new float[2];

    @Override
    public Mat processFrame(Mat input) {
        // Convert mat into HSV
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);

        Core.inRange(mat, lowHSV, highHSV, mat);

        erosionKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(8, 7), new Point(-1, -1));
        Imgproc.erode(mat, mat, erosionKernel);
        Imgproc.dilate(mat, mat, erosionKernel);

        input.copyTo(boxedInput);

        maxVal = 0.0;
        maxIndex = -1;
        float maxVal = -1.0f;
        float objectAmount = (float) object;
        for (int i = 0; i < 2; i++) {
            Imgproc.rectangle(
                    boxedInput,
                    new Point((double) (i * input.cols()) / 2, 0),
                    new Point((double) ((i + 1) * input.cols()) / 2, input.rows()),
                    new Scalar(0, 0, 255), 4
            );

            cropped = mat.submat(
                    new Range(0, input.rows()),
                    new Range(i * input.cols() / 2, (i + 1) * input.cols() / 2)
            );

            sumValue = Core.sumElems(cropped);
            totalPixels = (float) (cropped.cols() * cropped.rows());
            sumValNorm[i] = (float) (sumValue.val[0]) / totalPixels / 255.0f;

            if (sumValNorm[i] > maxVal) {
                maxVal = sumValNorm[i];
                maxIndex = i;
            }
        }

        if (maxVal < objectAmount) {
            maxIndex = 2;
        }

        Imgproc.putText(boxedInput, Integer.toString(maxIndex), new Point(50, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3, new Scalar(0, 0, 255), 22);

        return boxedInput;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

//    @Override
//    public Randomization getPosition() {
//        return position;
//    }
}
