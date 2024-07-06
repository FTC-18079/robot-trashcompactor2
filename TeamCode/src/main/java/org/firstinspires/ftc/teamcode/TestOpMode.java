package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Test OpMode", group = "Tests")
public class TestOpMode extends LinearOpMode {

    private DcMotorEx fl;
    private DcMotorEx bl;
    private DcMotorEx fr;
    private DcMotorEx br;

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        br.setDirection(DcMotorSimple.Direction.REVERSE);

        while(opModeIsActive() || !isStopRequested()) {
            double input = responseCurve(-gamepad1.left_stick_y, 1.1);

            fl.setPower(input);
            bl.setPower(input);
            fr.setPower(input);
            br.setPower(input);

            telemetry.addData("FL", fl.getVelocity(AngleUnit.DEGREES) / 6);
            telemetry.addData("BL", bl.getVelocity(AngleUnit.DEGREES) / 6);
            telemetry.addData("FR", fr.getVelocity(AngleUnit.DEGREES) / 6);
            telemetry.addData("BR", br.getVelocity(AngleUnit.DEGREES) / 6);

            double loop = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (loop - loopTime));
            telemetry.addData("Runtime", endTime == 0 ? timer.seconds() : endTime);
            loopTime = loop;

            telemetry.update();
        }
    }

    public double responseCurve(double value, double power) {
//        value = deadzone(value, DEADZONE);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }
}
