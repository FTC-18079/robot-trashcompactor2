package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Autonomous(name = "Test auto Blue", group = "Tests")
public class TestAuto extends OpMode {
    private Follower follower;

    // Start Pose
    Pose startPose = new Pose(135.5, 84, Math.toRadians(180));
    // Spike mark locations
    Pose leftSpikeMark = new Pose(105.5, 83, Math.toRadians(270));
    Pose centerSpikeMark = new Pose(107, 84, Math.toRadians(180));
    Pose rightSpikeMark = new Pose(107, 105, Math.toRadians(270));
    // Backdrop locations
    Pose leftBackdrop = new Pose(102, 121.75, Math.toRadians(90));
    Pose centerBackdrop = new Pose(108, 121.75, Math.toRadians(90));
    Pose rightBackdrop = new Pose(114.5, 121.75, Math.toRadians(90));
    Pose dumpBackdrop = new Pose(108, 121.75, Math.toRadians(90));

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }
}
