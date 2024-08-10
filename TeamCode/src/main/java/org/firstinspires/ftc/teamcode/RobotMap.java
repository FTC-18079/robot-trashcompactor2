package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

public class RobotMap {
    private HardwareMap hMap;
    // Sensors
    public VoltageSensor VOLTAGE;
    public WebcamName CAMERA_AT;
    public WebcamName CAMERA_OBJECT;
    public SparkFunOTOS OTOS;

    // Drive Motors
    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BR;

    // Intake
    public MotorEx COLLECTOR;
    public MotorEx RAMP;
    public CRServo FEEDER;

    // Plate movers
    public Servo PLATE_LEFT;
    public Servo PLATE_RIGHT;
    // Shooter
    public MotorEx PIVOT;
    public MotorEx SHOOTER;
    public Servo FLICK;
    public Servo SEAL;

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
        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");

        MOTOR_FL = hardwareMap.get(DcMotorEx.class, "leftFront");
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, "leftRear");
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, "rightFront");
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, "rightRear");

        COLLECTOR = new MotorEx(hardwareMap, "collector");
        RAMP = new MotorEx(hardwareMap, "ramp");
        FEEDER = hardwareMap.get(CRServo.class, "feeder");

        PLATE_RIGHT = hardwareMap.get(Servo.class, "plateRight");
        PLATE_LEFT = hardwareMap.get(Servo.class, "plateLeft");

        PIVOT = new MotorEx(hardwareMap, "pivot");
        SHOOTER = new MotorEx(hardwareMap, "shooter", 146.44, 1147.23);
        FLICK = hardwareMap.get(Servo.class, "flick");
        SEAL = hardwareMap.get(Servo.class, "seal");

        this.hMap = hardwareMap;
    }

    public List<LynxModule> getLynxModules() {
        return hMap.getAll(LynxModule.class);
    }

    public int getMonitorId() {
        return hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
    }

    public HardwareMap getHardwareMap() {
        return this.hMap;
    }
}
