package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    public DcMotorEx PAR0;
    public DcMotorEx PAR1;
    public DcMotorEx PERP;

    // Intake
    public MotorEx COLLECTOR;
    public MotorEx FEEDER;

    // Plate movers
    public Servo PLATE_LEFT;
    public Servo PLATE_RIGHT;
    public Servo PLATE_REAR;

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

        PAR0 = hardwareMap.get(DcMotorEx.class, "frontLeft");
        PAR1 = hardwareMap.get(DcMotorEx.class, "backRight");
        PERP = hardwareMap.get(DcMotorEx.class, "backLeft");

        COLLECTOR = new MotorEx(hardwareMap, "collector");
//        FEEDER = new MotorEx(hardwareMap, "feeder");

        PLATE_RIGHT = hardwareMap.get(Servo.class, "plateRight");
        PLATE_REAR = hardwareMap.get(Servo.class, "plateRear");
        PLATE_LEFT = hardwareMap.get(Servo.class, "plateLeft");

        this.hMap = hardwareMap;
    }

    public int getMonitorId() {
        return hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
    }

    public HardwareMap getHardwareMap() {
        return this.hMap;
    }
}
