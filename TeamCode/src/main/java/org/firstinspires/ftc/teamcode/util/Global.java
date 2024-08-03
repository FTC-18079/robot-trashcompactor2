package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.vision.DetectionPipeline;

public class Global {
    public enum Alliance {
        RED, BLUE, NONE
    }
    public enum Randomization {
        LEFT, CENTER, RIGHT
    }

    public static long delayMs;
    public static Field2d field = new Field2d();
    public static Alliance alliance = Alliance.NONE;
    public static DetectionPipeline.Position randomization;
}
