package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class AutoConstants {
    public static double startingX = 12.0;
    public static double startingY = -61.34;
    public static double startingH = 90.0;
    public static final Pose2d startingPose = new Pose2d(startingX, startingY, startingH);
}
