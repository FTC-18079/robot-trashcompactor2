package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
public class BluePipeline extends Pipeline {//, PipelineIF {
    private volatile Randomization position = Randomization.RIGHT;

    private Scalar lowHSV;
    private Scalar highHSV;
    private Scalar color;

    // Mats we need
    private Mat firstCrop = new Mat();
    private Mat erosionKernel = new Mat();
    private Mat bgrMat = new Mat();             // input in BGR
    private Mat hsvMat = new Mat();             // BGR mat in HSV format
    private Mat binaryMat = new Mat();          // filters out everything to black and white
    private Mat erodedMat = new Mat();          // removes small pixels to further filter
    private Mat boxedInput = new Mat();
    private Mat cropped = new Mat();

    double maxVal = 0.0;
    int maxIndex = -1;
    public static double object = 0.002;
    Scalar sumValue = new Scalar(0);
    float totalPixels = 0.0f;
    float[] sumValNorm = new float[2];
    Telemetry telemetry;

    public BluePipeline(Telemetry telemetry) {
        if (Global.alliance == Global.Alliance.BLUE) {
            lowHSV = VisionConstants.lowBlueHSV;
            highHSV = VisionConstants.highBlueHSV;
            color = new Scalar(0, 0, 255);
        } else {
            lowHSV = VisionConstants.lowRedHSV;
            highHSV = VisionConstants.highRedHSV;
            color = new Scalar(255, 0, 0);
        }
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        input.submat(
                new Range((int) (0.33 * input.rows()), input.rows()),
                new Range(0, input.cols())
        ).copyTo(firstCrop);

        // Convert mat into HSV
        Imgproc.cvtColor(firstCrop, bgrMat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(bgrMat, hsvMat, Imgproc.COLOR_BGR2HSV);

        Core.inRange(hsvMat, lowHSV, highHSV, binaryMat);

        Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(8, 7), new Point(-1, -1)).copyTo(erosionKernel);
        Imgproc.erode(binaryMat, erodedMat, erosionKernel);
        Imgproc.dilate(erodedMat, erodedMat, erosionKernel);

        firstCrop.copyTo(boxedInput);

        maxVal = 0.0;
        maxIndex = -1;
        float maxVal = -1.0f;
        float objectAmount = (float) object;
        for (int i = 0; i < 2; i++) {
            Imgproc.rectangle(
                    boxedInput,
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

            if (sumValNorm[i] > maxVal) {
                telemetry.addData("SUM", sumValNorm[i]);
                telemetry.addData("VAL", maxVal);
                maxVal = sumValNorm[i];
                maxIndex = i;
            }
        }

        if (maxVal < objectAmount) {
            telemetry.addData("OBJECT", objectAmount);
            telemetry.addData("VAL", maxVal);
            maxIndex = 2;
        }

        if (maxIndex == 0) position = Randomization.LEFT;
        else if (maxIndex == 1) position = Randomization.CENTER;
        else position = Randomization.RIGHT;

        Imgproc.putText(boxedInput, Integer.toString(maxIndex), new Point(50, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3, color, 22);

        return boxedInput;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public Randomization getPosition() {
        return position;
    }
}
