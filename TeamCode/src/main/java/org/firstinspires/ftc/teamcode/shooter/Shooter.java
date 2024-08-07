package org.firstinspires.ftc.teamcode.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;

import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    Telemetry telemetry;
    Servo plateLeft;
    Servo plateRight;
    boolean inShootingMode;
    double pivotAngle;

    public Shooter(RobotCore robot) {
        this.telemetry = robot.getTelemetry();

        plateLeft = RobotMap.getInstance().PLATE_LEFT;
        plateRight = RobotMap.getInstance().PLATE_RIGHT;
    }

    public void shoot() {
        // fire.setPosition(1)
    }

    public void closeSeal() {

    }

    public void openSeal() {

    }

    public void readyShooter() {
        // shooter.setVelocity
    }

    public void stopShooter() {
        // shooter.stop()
    }

    public boolean isReady() {
        // return shooter.atTargetVelocity() && pivot.atTargetAngle()
        return true;
    }

    public boolean pivotReady() {
        // return pivot.atTargetAngle()
        return true;
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

    public void aimPivot(double angle) {
        // pivot.setTargetPosition(targetAngle)
    }

    public void pivotDown() {
        pivotAngle = 0.0;
    }

    public boolean isInShootingMode() {
        return inShootingMode;
    }

    @Override
    public void periodic() {
        aimPivot(pivotAngle);
        if (inShootingMode) {
            // pivotAngle = calculatePivotAngle
        }

        telemetry.addData("In Shooting Mode", inShootingMode);
    }
}
