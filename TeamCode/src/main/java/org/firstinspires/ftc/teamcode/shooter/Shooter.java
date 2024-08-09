package org.firstinspires.ftc.teamcode.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;

import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    Telemetry telemetry;
    // Plate servos
    Servo plateLeft;
    Servo plateRight;
    // Cannon
    MotorEx pivot;
    MotorEx shooter;
    Servo flick;
    Servo seal;

    boolean inShootingMode;
    int pivotAngle;

    public Shooter(RobotCore robot) {
        this.telemetry = robot.getTelemetry();

        plateLeft = RobotMap.getInstance().PLATE_LEFT;
        plateRight = RobotMap.getInstance().PLATE_RIGHT;

        pivot = RobotMap.getInstance().PIVOT;
        shooter = RobotMap.getInstance().SHOOTER;
        flick = RobotMap.getInstance().FLICK;

        inShootingMode = false;
        setupMotors();
    }

    public void setupMotors() {
        pivot.stopAndResetEncoder();
        pivot.setInverted(false);
        pivot.setPositionCoefficient(PIVOT.kP);
        pivot.setPositionTolerance(PIVOT.POSITION_TOLERANCE);
        pivot.setRunMode(Motor.RunMode.PositionControl);
        pivot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        shooter.stopAndResetEncoder();
        shooter.setInverted(false);
        shooter.setVeloCoefficients(LAUNCHER.kP, LAUNCHER.kI, LAUNCHER.kD);
        shooter.setFeedforwardCoefficients(LAUNCHER.kS, LAUNCHER.kV, LAUNCHER.kA);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void shoot() {
        flick.setPosition(1);
    }

    public void flickBack() {
        flick.setPosition(0);
    }

    public void closeSeal() {

    }

    public void openSeal() {

    }

    public void readyShooter() {
         shooter.setVelocity(toTicksPerSec(LAUNCHER.RPM, (int) shooter.getCPR()));
    }

    public void stopShooter() {
        shooter.stopMotor();
    }

    public boolean isReadyToFire() {
        double velocity = shooter.getCorrectedVelocity();
        return Math.abs(LAUNCHER.RPM - velocity) < LAUNCHER.VELOCITY_TOLERANCE && pivotReady();
    }

    public boolean pivotReady() {
        return pivot.atTargetPosition();
    }

    public void toggleShootingMode() {
        inShootingMode = !inShootingMode;
    }

    public void plateOut() {
        plateLeft.setPosition(PLATE.LEFT_PLATE_OUT);
        plateRight.setPosition(PLATE.RIGHT_PLATE_OUT);
    }

    public void plateStow() {
        plateLeft.setPosition(PLATE.LEFT_PLATE_STOW);
        plateRight.setPosition(PLATE.RIGHT_PLATE_STOW);
    }

    public void aimPivot(int angle) {
         pivot.setTargetPosition(angle);
    }

    public void pivotDown() {
        pivotAngle = 0;
    }

    public boolean isInShootingMode() {
        return inShootingMode;
    }

    public int calculatePivotAngle() {
        return 0;
    }

    @Override
    public void periodic() {
        if (inShootingMode) pivotAngle = calculatePivotAngle();

        aimPivot(pivotAngle);
        pivot.set(PIVOT.RPM);

        telemetry.addData("In Shooting Mode", inShootingMode);
    }
}
