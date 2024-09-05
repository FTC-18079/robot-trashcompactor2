package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

// FTCLib Command wrapper for following paths
public class FollowPathCommand extends CommandBase {
    RobotCore robot;
    Follower follower;
    Path path;

    public FollowPathCommand(RobotCore robot, Path path) {
        this.robot = robot;
        this.follower = robot.getFollower();
        this.path = path;
    }

    @Override
    public void initialize() {
        follower.followPath(path);
    }

    @Override
    public boolean isFinished() {
        return follower.isBusy();
    }
}
