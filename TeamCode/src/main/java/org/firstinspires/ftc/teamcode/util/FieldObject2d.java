// Taken from WPILib, modified for FTC

package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Game field object on a Field2d.
 * Taken from WPILib
 */

public class FieldObject2d {
    String name;
    double[] entry;
    private final List<Pose2d> poses = new ArrayList<Pose2d>();

    FieldObject2d(String name) {
        this.name = name;
    }

    /**
     * Set the pose from a Pose object.
     *
     * @param pose 2D pose
     */
    public synchronized void setPose(Pose2d pose) {
        setPoses(pose);
    }

    /**
     * Set the pose from x, y, and rotation.
     *
     * @param xMeters X location, in meters
     * @param yMeters Y location, in meters
     * @param rotation rotation
     */
    public synchronized void setPose(double xMeters, double yMeters, Rotation2d rotation) {
        setPose(new Pose2d(xMeters, yMeters, rotation));
    }

    /**
     * Get the pose.
     *
     * @return 2D pose
     */
    public synchronized Pose2d getPose() {
        if (this.poses.isEmpty()) {
            return new Pose2d(0, 0, new Rotation2d(0));
        }
        return this.poses.get(0);
    }

    /**
     * Set multiple poses from a list of Pose objects. The total number of poses is limited to 85.
     *
     * @param poses list of 2D poses
     */
    public synchronized void setPoses(List<Pose2d> poses) {
        this.poses.clear();
        this.poses.addAll(poses);
    }

    /**
     * Set multiple poses from a list of Pose objects. The total number of poses is limited to 85.
     *
     * @param poses list of 2D poses
     */
    public synchronized void setPoses(Pose2d... poses) {
        this.poses.clear();
        Collections.addAll(this.poses, poses);
    }

    /**
     * Sets poses from a trajectory.
     *
     * @param trajectory The trajectory from which the poses should be added.
     */
    public synchronized void setTrajectory(Trajectory trajectory) {
        poses.clear();
        for (Trajectory.State state : trajectory.getStates()) {
            poses.add(new Pose2d(
                    state.poseMeters.getX(),
                    state.poseMeters.getY(),
                    new Rotation2d(state.poseMeters.getHeading())
            ));
        }
    }

    /**
     * Get multiple poses.
     *
     * @return list of 2D poses
     */
    public synchronized List<Pose2d> getPoses() {
        return new ArrayList<>(poses);
    }
}
