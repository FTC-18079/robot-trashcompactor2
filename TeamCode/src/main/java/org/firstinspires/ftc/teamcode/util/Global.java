package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.vision.PipelineIF;

public class Global {

    public enum Alliance {
        RED, BLUE, NONE
    }

    public static long delayMs;

    public static boolean liveView;
    public static Field2d field = new Field2d();
    public static Alliance alliance = Alliance.NONE;
    public static PipelineIF.Randomization randomization;
}
