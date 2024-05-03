package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HardwareTest extends LinearOpMode {
    public static String MOTOR_NAME = "driveFL";
    public static String SERVO_NAME = "";
    DcMotorEx motor;
    Servo servo;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
//        servo = hardwareMap.get(Servo.class, SERVO_NAME);

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
        }
    }
}
