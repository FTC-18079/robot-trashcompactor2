package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.vision.DetectionPipeline;

public class Global {
    public enum Alliance {
        RED, BLUE, NONE
    }
    public static Field2d field = new Field2d();
    public static Alliance alliance;
    public static DetectionPipeline.Position randomization;
}
