package org.firstinspires.ftc.teamcode.util.vision;

public interface PipelineIF {
    enum Randomization {
        LEFT, CENTER, RIGHT
    }
    Randomization getPosition();
}
