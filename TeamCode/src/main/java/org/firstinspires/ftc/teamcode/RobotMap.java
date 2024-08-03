package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class RobotMap {
    private HardwareMap hMap;
    // Sensors
    public VoltageSensor VOLTAGE;
    public WebcamName CAMERA_AT;
    public WebcamName CAMERA_OBJECT;

    // Drive Motors
    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BR;

    // Odometry pods
    public DcMotorEx par0;
    public DcMotorEx par1;
    public DcMotorEx perp;

    // Manipulators
    public static final String PIVOT = "pivot";
    public static final String SHOOTER = "shooter";
    public static final String INDEXER = "indexer";

    private static RobotMap instance = null;

    public static RobotMap getInstance() {
        if (instance == null) {
            instance = new RobotMap();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        VOLTAGE = hardwareMap.voltageSensor.iterator().next();
        CAMERA_AT = hardwareMap.get(WebcamName.class, "arducam");
        CAMERA_OBJECT = hardwareMap.get(WebcamName.class, "object");

        MOTOR_FL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, "backLeft");
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, "frontRight");
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, "backRight");

        par0 = hardwareMap.get(DcMotorEx.class, "frontLeft");
        par1 = hardwareMap.get(DcMotorEx.class, "backRight");
        perp = hardwareMap.get(DcMotorEx.class, "backLeft");

        this.hMap = hardwareMap;
    }

    public int getMonitorId() {
        return hMap.appContext.getResources().getIdentifier("cameraMonitorId", "id", hMap.appContext.getPackageName());
    }

    public HardwareMap getHardwareMap() {
        return this.hMap;
    }
}
