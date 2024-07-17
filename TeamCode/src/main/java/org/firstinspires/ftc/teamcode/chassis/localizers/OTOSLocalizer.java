package org.firstinspires.ftc.teamcode.chassis.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;

@Config
public class OTOSLocalizer implements Localizer {
    public static double linearScalar = 1.0;
    public static double angularScalar = 1.0;

    public final SparkFunOTOS otos;

    private double lastX, lastY, lastH;
    private boolean initialized;

    public OTOSLocalizer(HardwareMap hMap) {
        otos = hMap.get(SparkFunOTOS.class, RobotMap.OTOS);
        configureOtos();
    }

    private void configureOtos() {
        //Units
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        // TODO: calibrate that jawn
        //Scalars (Calibrate Angular, then Linear)
        otos.setLinearScalar(linearScalar);
        otos.setAngularScalar(angularScalar);

        // TODO: Add real positions
        //Right is positive x, Forward is positive y, clockwise is negative rotation
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0,0,0);
        otos.setOffset(offset);

        otos.calibrateImu();
        otos.resetTracking();
    }

    public void setPosition(Pose2d pose2d) {
        // :skull:
        otos.setPosition(new SparkFunOTOS.Pose2D(pose2d.position.x, pose2d.position.y, pose2d.heading.real));
    }

    @Override
    public Twist2dDual<Time> update() {
        SparkFunOTOS.Pose2D readPos = otos.getPosition();
        SparkFunOTOS.Pose2D readVel = otos.getVelocity();
        if (!initialized) {
            initialized = true;

            lastX = readPos.x;
            lastY = readPos.y;
            lastH = readPos.h;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        double xDelta = readPos.x - lastX;
        double yDelta = readPos.y - lastY;
        double hDelta = readPos.h - lastH;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                readPos.x, readVel.x
                        }),
                        new DualNum<Time>(new double[] {
                                readPos.y, readVel.y
                        })
                ),
                new DualNum<>(new double[] {
                        readPos.h, readVel.h
                })
        );

        lastX = readPos.x;
        lastY = readPos.y;
        lastH = readPos.h;

        return twist;
    }
}
