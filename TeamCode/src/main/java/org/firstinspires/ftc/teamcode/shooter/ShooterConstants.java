package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

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
        public double RPM = -6000.0;
        public double VELOCITY_TOLERANCE = 25.0;
        // PID
        public double kP = 1.0;
        public double kI = 0.0;
        public double kD = 0.0;
        // FeedForward
        public double kS = 0.0;
        public double kV = 1.3;
        public double kA = 0.0;
    }

    public static class Pivot {
        public double RPM = 0.5;
        public double POSITION_TOLERANCE = 15.0;
        public int MAX_ANGLE = -550;
        // PID
        public double kP = 0.0005;
        public double kI = 0.0;
        public double kD = 0.0;
    }

    public static class Seal {
        public double CLOSE_POS = 1.0;
        public double OPEN_POS = 0.8;
    }

    public static double GOAL_X = -72.0;
    public static double GOAL_Y = 48.0;

    public static Plate PLATE = new Plate();
    public static Launcher LAUNCHER = new Launcher();
    public static Pivot PIVOT = new Pivot();
    public static Seal SEAL = new Seal();
    public static Pose GOAL_POSE = new Pose(GOAL_X, GOAL_Y);

    public static double toTicksPerSec(double rpm, double motorTicks) {
        return (rpm / 60) * motorTicks;
    }

    public static double toRPM(double tps, double motorTicks) {
        return (tps / motorTicks) * 60.0;
    }

    public static double angleToTicks(double degrees, double cpr) {
        return (degrees / 360.0) * cpr;
    }

    public static double ticksToAngle(double ticks, double cpr) {
        return (ticks / cpr) * 360.0;
    }
}
