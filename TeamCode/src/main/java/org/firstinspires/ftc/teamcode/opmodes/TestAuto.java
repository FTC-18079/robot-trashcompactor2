package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.vision.PipelineIF;

@Autonomous(name = "Test auto Blue", group = "Tests")
public class TestAuto extends OpMode {
    private Follower follower;
    private int pathState, actionState;
    Timer pathTimer, opmodeTimer;
    private PipelineIF.Randomization randomization;

    // Start Pose
    private Pose startPose = new Pose(135.5, 84, Math.toRadians(180));
    // Spike mark locations
    private Pose leftSpikeMark = new Pose(105.5, 83, Math.toRadians(270));
    private Pose centerSpikeMark = new Pose(107, 84, Math.toRadians(180));
    private Pose rightSpikeMark = new Pose(107, 105, Math.toRadians(270));
    // Backdrop locations
    private Pose leftBackdrop = new Pose(102, 121.75, Math.toRadians(90));
    private Pose centerBackdrop = new Pose(108, 121.75, Math.toRadians(90));
    private Pose rightBackdrop = new Pose(114.5, 121.75, Math.toRadians(90));
    private Pose dumpBackdrop = new Pose(108, 121.75, Math.toRadians(90));
    // Poses for paths
    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleGoalPose;
    private Path scoreSpikeMark, initialScoreOnBackdrop, scoreSpikeMarkChosen;

    public void setPathRandomization() {
        switch (randomization) {
            case LEFT:
                spikeMarkGoalPose = new Pose(leftSpikeMark.getX(), leftSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(leftBackdrop.getX(), leftBackdrop.getY(), Math.toRadians(90));
                firstCycleGoalPose = new Pose(dumpBackdrop.getX(), dumpBackdrop.getY(), Math.toRadians(90));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(leftSpikeMark)));
                break;
            case CENTER:
                spikeMarkGoalPose = new Pose(centerSpikeMark.getX(), centerSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(centerBackdrop.getX(), centerBackdrop.getY(), Math.toRadians(90));
                firstCycleGoalPose = new Pose(dumpBackdrop.getX(), dumpBackdrop.getY(), Math.toRadians(90));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(centerSpikeMark)));
                break;
            default:
                spikeMarkGoalPose = new Pose(rightSpikeMark.getX(), rightSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(rightBackdrop.getX(), rightBackdrop.getY(), Math.toRadians(90));
                firstCycleGoalPose = new Pose(dumpBackdrop.getX(), dumpBackdrop.getY(), Math.toRadians(90));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(rightSpikeMark)));
                break;
        }
    }

    public void buildPaths() {
        scoreSpikeMark = scoreSpikeMarkChosen;
        scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading(), spikeMarkGoalPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(0);

        initialScoreOnBackdrop = new Path(new BezierLine(new Point(spikeMarkGoalPose), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setLinearHeadingInterpolation(spikeMarkGoalPose.getHeading(), initialBackdropGoalPose.getHeading());
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(scoreSpikeMark);
                setPathState(11);
                break;
        }
    }

    public void autonomousActionUpdate() {

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    public void setActionState(int aState) {
        actionState = aState;
        pathTimer.resetTimer();
        autonomousActionUpdate();
    }

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
