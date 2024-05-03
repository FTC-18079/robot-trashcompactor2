package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Hardware Test", group = "Util")
public class HardwareTest extends LinearOpMode {
    public static String MOTOR_NAME = "driveFL";
    public static boolean BRAKES_ENABLED = true;
    public static String SERVO_NAME = "";
    DcMotorEx motor;
    Servo servo;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Motor to test", MOTOR_NAME);
        telemetry.addData("Servo to test", SERVO_NAME);
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            motor.setPower(-gamepad1.left_stick_y);

            telemetry.addData("Motor power", motor.getPower());
            telemetry.addData("Motor velocity", motor.getVelocity());
            telemetry.update();

            if (gamepad1.a) initHardware();
        }
    }

    public void initHardware() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setZeroPowerBehavior(BRAKES_ENABLED ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);

//        servo = hardwareMap.get(Servo.class, SERVO_NAME);
    }
}
