package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Global {
    public enum Alliance {
        RED, BLUE
    }
    public static Pose2d robotPose = new Pose2d(new Vector2d(0, 0), 0);
    public static Alliance alliance;
    public static int randomization;
}
