package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;

@Config
public class Shooter extends SubsystemBase {
    RobotCore robot;
    Telemetry telemetry;
    // Plate servos
    Servo plateLeft;
    Servo plateRight;
    // Cannon
    MotorEx pivot;
    MotorEx shooter;
    Servo flick;
    Servo seal;
    // Fields
    boolean inShootingMode;
    int pivotAngle;
    double shooterVelocity;

    //TODO: temporary, remove this
    public static int PIVOT_ANGLE = -500;

    public Shooter(RobotCore robot) {
        this.robot = robot;
        this.telemetry = robot.getTelemetry();

        plateLeft = RobotMap.getInstance().PLATE_LEFT;
        plateRight = RobotMap.getInstance().PLATE_RIGHT;

        pivot = RobotMap.getInstance().PIVOT;
        shooter = RobotMap.getInstance().SHOOTER;
        flick = RobotMap.getInstance().FLICK;
        seal = RobotMap.getInstance().SEAL;

        inShootingMode = false;
        pivotAngle = 0;
        shooterVelocity = 0.0;
        setupMotors();
    }

    public void setupMotors() {
        pivot.stopAndResetEncoder();
        pivot.setInverted(false);
        pivot.setPositionCoefficient(PIVOT.kP);
        pivot.setPositionTolerance(PIVOT.POSITION_TOLERANCE);
        pivot.resetEncoder();
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
        flick.setPosition(0.3);
    }

    public void flickBack() {
        flick.setPosition(0);
    }

    public void closeSeal() {
        seal.setPosition(SEAL.CLOSE_POS);
    }

    public void openSeal() {
        seal.setPosition(SEAL.OPEN_POS);
    }

    public void readyShooter() {
        shooterVelocity = toTicksPerSec(LAUNCHER.RPM, shooter.getCPR());
    }

    public void stopShooter() {
        shooterVelocity = 0.0;
    }

    public boolean isReadyToFire() {
        double velocity = toRPM(shooter.getCorrectedVelocity(), shooter.getCPR());
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

    public void setPivot() {
        if (pivotReady()) pivot.set(0);
        else pivot.set(toTicksPerSec(PIVOT.RPM, pivot.getCPR()));
    }

    public boolean isInShootingMode() {
        return inShootingMode;
    }

    // TODO: calc the pivot angle formula
    // pivot will work off motor ticks rather than angle (for now)
    public int calculatePivotAngle() {
        // Calculate distance to goal
        Pose pose = robot.getPoseEstimate();
        double dist = MathFunctions.distance(pose, GOAL_POSE);

        return (int) PIVOT_ANGLE;
    }

    @Override
    public void periodic() {
        if (inShootingMode) pivotAngle = (int) MathFunctions.clamp(calculatePivotAngle(), PIVOT.MAX_ANGLE, 0);
        else pivotAngle = 0;

        shooter.setVelocity(shooterVelocity);
        aimPivot(pivotAngle);
        setPivot();

        telemetry.addData("In Shooting Mode", inShootingMode);

        // TODO: remove this, it's debug
        telemetry.addData("Target shooter RPM", toRPM(shooterVelocity, shooter.getCPR()));
        telemetry.addData("Shooter RPM", toRPM(shooter.getCorrectedVelocity(), shooter.getCPR()));
        telemetry.addData("Shooter TPS", shooter.getCorrectedVelocity());

        telemetry.addData("Target pivot pos", calculatePivotAngle());
        telemetry.addData("Pivot pos", pivot.getCurrentPosition());
    }
}
