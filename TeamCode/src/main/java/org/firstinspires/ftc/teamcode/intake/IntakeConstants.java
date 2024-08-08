package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {
    public static class Collector {
        public double RPM = 250.0;
        // PID
        public double kP = 0.005;
        public double kI = 0.0;
        public double kD = 0.01;

        // Feedforward
        public double kS = 0.0;
        public double kV = 1.05;
        public double kA = 0.0;
    }

    public static class Ramp {
        public double RPM = 250.0;
        // PID
        public double kP = 0.2;
        public double kI = 0.0;
        public double kD = 0.0;

        // Feedforward
        public double kS = 0.0;
        public double kV = 0.885;
        public double kA = 0.0;
    }

    public static Collector COLLECTOR = new Collector();
    public static Ramp RAMP = new Ramp();

    public static int MOTOR_TICKS = 560;

    public static double toTicksPerSec(double rpm) {
        return (rpm / 60) * (double) MOTOR_TICKS;
    }

    public static double toRPM(double tps) {
        return (tps / (double) MOTOR_TICKS) * 60.0;
    }
}
