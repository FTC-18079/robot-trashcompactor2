package org.firstinspires.ftc.teamcode.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;

import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
    Telemetry telemetry;
    MotorEx collector;
    MotorEx ramp;
    CRServo feeder;

    public Intake(RobotCore robot) {
        this.telemetry = robot.getTelemetry();
        collector = RobotMap.getInstance().COLLECTOR;
        ramp = RobotMap.getInstance().RAMP;
        feeder = RobotMap.getInstance().FEEDER;

        setupMotors();
    }

    public void setupMotors() {
        collector.stopAndResetEncoder();
        collector.setInverted(false);
        collector.setVeloCoefficients(COLLECTOR.kP, COLLECTOR.kI, COLLECTOR.kD);
        collector.setFeedforwardCoefficients(COLLECTOR.kS, COLLECTOR.kV, COLLECTOR.kA);
        collector.setRunMode(Motor.RunMode.VelocityControl);
        collector.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        ramp.stopAndResetEncoder();
        ramp.setInverted(true);
        ramp.setVeloCoefficients(RAMP.kP, RAMP.kI, RAMP.kD);
        ramp.setFeedforwardCoefficients(RAMP.kS, RAMP.kV, RAMP.kA);
        ramp.setRunMode(Motor.RunMode.VelocityControl);
        ramp.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void in() {
        collector.setVelocity(toTicksPerSec(-COLLECTOR.RPM));
        ramp.setVelocity(toTicksPerSec(RAMP.RPM));
        feeder.setPower(1);
    }

    public void eject() {
        collector.setVelocity(toTicksPerSec(COLLECTOR.RPM));
        ramp.setVelocity(toTicksPerSec(-RAMP.RPM));
        feeder.setPower(-1);
    }

    public void stop() {
        collector.stopMotor();
        ramp.stopMotor();
        feeder.setPower(0);
    }

    @Override
    public void periodic() {
        telemetry.addData("Collector RPM", Math.abs(toRPM(collector.getCorrectedVelocity())));
        telemetry.addData("Ramp RPM", Math.abs(toRPM(ramp.getCorrectedVelocity())));
    }
}
