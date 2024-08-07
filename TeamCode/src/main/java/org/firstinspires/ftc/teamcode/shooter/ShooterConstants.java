package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static class Plate {
        // Left plate poses
        public double LEFT_PLATE_STOW = 1.0;
        public double LEFT_PLATE_OUT = 0.0;
        // Right plate poses
        public double RIGHT_PLATE_STOW = 0.0;
        public double RIGHT_PLATE_OUT = 1.0;
    }

    public static class Launcher {
        public double RPM = 0.0;
    }

    public static class Pivot {

    }

    public static Plate PLATE = new Plate();
    public static Launcher LAUNCHER = new Launcher();
    public static Pivot PIVOT = new Pivot();
}
