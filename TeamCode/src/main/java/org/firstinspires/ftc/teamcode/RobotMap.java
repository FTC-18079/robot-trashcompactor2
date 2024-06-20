package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.chassis.kinematics.MecanumDrive;

public class RobotMap {
    // Sensors
    public LazyImu imu;
    public WebcamName tagCamera;
    public WebcamName objectCamera;

    // Drivetrain
    public DcMotorEx driveFL;
    public DcMotorEx driveBL;
    public DcMotorEx driveFR;
    public DcMotorEx driveBR;

    // Odometry
    public OverflowEncoder par0;
    public OverflowEncoder par1;
    public OverflowEncoder perp;

    // Manipulators
    public DcMotorEx pivot;
    public DcMotorEx shooter;
    public DcMotorEx indexer;

    private static RobotMap INSTANCE = null;

    public static RobotMap getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotMap();
        }
        return INSTANCE;
    }

    public void init(HardwareMap hardwareMap) {
        this.imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection));
        this.tagCamera = hardwareMap.get(WebcamName.class, "arducam");
        this.objectCamera = hardwareMap.get(WebcamName.class, "object");

        this.driveFL = hardwareMap.get(DcMotorEx.class, "driveFL");
        this.driveFR = hardwareMap.get(DcMotorEx.class, "driveFR");
        this.driveBL = hardwareMap.get(DcMotorEx.class, "driveBL");
        this.driveBR = hardwareMap.get(DcMotorEx.class, "driveBR");

        this.par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "driveFL")));
        this.par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "driveBR")));
        this.perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "driveBL")));

//        this.pivot = hardwareMap.get(DcMotorEx.class, "pivot");
//        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        this.shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        this.indexer = hardwareMap.get(DcMotorEx.class, "indexer");
//        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
