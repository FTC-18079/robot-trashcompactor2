// Taken from WPILib, modified for FTC

package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Game field object on a Field2d.
 * Taken from WPILib
 */

public class FieldObject2d {
    String name;
    double[] entry;
    private final List<Pose> poses = new ArrayList<Pose>();

    FieldObject2d(String name) {
        this.name = name;
    }

    /**
     * Set the pose from a Pose object.
     *
     * @param pose 2D pose
     */
    public synchronized void setPose(Pose pose) {
        setPoses(pose);
    }

    /**
     * Set the pose from x, y, and rotation.
     *
     * @param xMeters X location, in meters
     * @param yMeters Y location, in meters
     * @param rotation rotation
     */
    public synchronized void setPose(double xMeters, double yMeters, double rotation) {
        setPose(new Pose(xMeters, yMeters, rotation));
    }

    /**
     * Get the pose.
     *
     * @return 2D pose
     */
    public synchronized Pose getPose() {
        if (this.poses.isEmpty()) {
            return new Pose(0, 0, 0);
        }
        return this.poses.get(0);
    }

    /**
     * Set multiple poses from a list of Pose objects. The total number of poses is limited to 85.
     *
     * @param poses list of 2D poses
     */
    public synchronized void setPoses(List<Pose> poses) {
        this.poses.clear();
        this.poses.addAll(poses);
    }

    /**
     * Set multiple poses from a list of Pose objects. The total number of poses is limited to 85.
     *
     * @param poses list of 2D poses
     */
    public synchronized void setPoses(Pose... poses) {
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
            poses.add(new Pose(
                    state.poseMeters.getX(),
                    state.poseMeters.getY(),
                    state.poseMeters.getHeading()
            ));
        }
    }

    /**
     * Get multiple poses.
     *
     * @return list of 2D poses
     */
    public synchronized List<Pose> getPoses() {
        return new ArrayList<>(poses);
    }
}
