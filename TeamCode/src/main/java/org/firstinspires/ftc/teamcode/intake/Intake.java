package org.firstinspires.ftc.teamcode.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;

import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
    Telemetry telemetry;
    MotorEx collector;
    MotorEx feeder;

    public Intake(RobotCore robot) {
        this.telemetry = robot.getTelemetry();
        collector = RobotMap.getInstance().COLLECTOR;
//        feeder = RobotMap.getInstance().FEEDER;

        setupMotors();
    }

    public void setupMotors() {
        collector.stopAndResetEncoder();
        collector.setInverted(false);
        collector.setVeloCoefficients(COLLECTOR.kP, COLLECTOR.kI, COLLECTOR.kD);
        collector.setFeedforwardCoefficients(COLLECTOR.kS, COLLECTOR.kV, COLLECTOR.kA);
        collector.setRunMode(Motor.RunMode.VelocityControl);
        collector.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

//        feeder.stopAndResetEncoder();
//        feeder.setInverted(false);
//        feeder.setVeloCoefficients(FEEDER.kP, FEEDER.kI, FEEDER.kD);
//        feeder.setFeedforwardCoefficients(FEEDER.kS, FEEDER.kV, FEEDER.kA);
//        feeder.setRunMode(Motor.RunMode.VelocityControl);
//        feeder.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void in() {
        collector.setVelocity(toTicksPerSec(-COLLECTOR.RPM));
//        feeder.setVelocity(FEEDER.RPM);
    }

    public void eject() {
        collector.setVelocity(toTicksPerSec(COLLECTOR.RPM));
//        feeder.setVelocity(-FEEDER.RPM);
    }

    public void stop() {
        collector.stopMotor();
//        feeder.stopMotor();
    }

    @Override
    public void periodic() {
        telemetry.addData("Collector RPM", Math.abs(toRPM(collector.getCorrectedVelocity())));
        telemetry.addData("Target collector RPM", COLLECTOR.RPM);
//        telemetry.addData("Feeder RPM", toRPM(feeder.getCorrectedVelocity()));
    }
}
