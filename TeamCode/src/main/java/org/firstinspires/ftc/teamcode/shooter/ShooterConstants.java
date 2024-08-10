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
        public double RPM = -315.0;
        public double VELOCITY_TOLERANCE = 10.0;
        // PID
        public double kP = 1.0;
        public double kI = 0.0;
        public double kD = 0.0;
        // FeedForward
        public double kS = 0.0;
        public double kV = 1.0;
        public double kA = 0.0;
    }

    public static class Pivot {
        public double RPM = -60.0;
        public double POSITION_TOLERANCE = 50.0;
        // PID
        public double kP = 1.0;
        public double kI = 0.0;
        public double kD = 0.0;
    }

    public static Plate PLATE = new Plate();
    public static Launcher LAUNCHER = new Launcher();
    public static Pivot PIVOT = new Pivot();

    public static double toTicksPerSec(double rpm, double motorTicks) {
        return (rpm / 60) * motorTicks;
    }

    public static double toRPM(double tps, double motorTicks) {
        return (tps / motorTicks) * 60.0;
    }
}
