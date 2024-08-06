package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import org.firstinspires.ftc.teamcode.util.vision.PipelineIF;
import org.firstinspires.ftc.teamcode.vision.pipelines.RedPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class VisionTest extends OpMode {

    private RedPipeline processor;
    private VisionPortal visionPortal;
//    PipelineIF.Randomization randomization;

    @Override
    public void init() {
        processor = new RedPipeline();
        //visionPortal = VisionPortal.easyCreateWithDefaults(RobotMap.getInstance().CAMERA_OBJECT, processor);
    }

    @Override
    public void init_loop() {
//        randomization = processor.getPosition();
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {

    }
}
