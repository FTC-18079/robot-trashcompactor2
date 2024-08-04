package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.vision.Pipeline;
import org.firstinspires.ftc.teamcode.util.vision.PipelineIF;
import org.firstinspires.ftc.teamcode.vision.pipelines.BlueDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.RedDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ObjectDetection {
    private OpenCvCamera objCamera;
    private Pipeline pipeline;

    public ObjectDetection(boolean liveView) {
        // Create camera
        if (liveView) objCamera = OpenCvCameraFactory.getInstance().createWebcam(RobotMap.getInstance().CAMERA_OBJECT, RobotMap.getInstance().getMonitorId());
        else objCamera = OpenCvCameraFactory.getInstance().createWebcam(RobotMap.getInstance().CAMERA_OBJECT);


        // Set pipeline
        if (Global.alliance == Global.Alliance.RED) pipeline = new RedDetectionPipeline();
        else pipeline = new BlueDetectionPipeline();

        // Start streaming
        objCamera.setPipeline(pipeline);
        objCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                objCamera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public PipelineIF.Randomization getPosition() {
        return pipeline.getPosition();
    }

    public void closeCamera() {
        objCamera.stopStreaming();
        objCamera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });
    }

    public double getFps() {
        return objCamera.getFps();
    }
}
