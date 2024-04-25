package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;

import java.util.ArrayList;
import java.util.List;

/**
 * 2D representation of game field for dashboards.
 *
 * <p>An object's pose is the location shown on the dashboard view. Note that for the robot, this
 * may or may not match the internal odometry. For example, the robot is shown at a particular
 * starting location, the pose in this class would represent the actual location on the field, but
 * the robot's internal state might have a 0,0,0 pose (unless it's initialized to something
 * different).
 *
 * <p>As the user is able to edit the pose, code performing updates should get the robot pose,
 * transform it as appropriate (e.g. based on wheel odometry), and set the new pose.
 *
 * <p>This class provides methods to set the robot pose, but other objects can also be shown by
 * using the getObject() function. Other objects can also have multiple poses (which will show the
 * object at multiple locations).
 */
public class Field2d {
    private final List<FieldObject2d> objects = new ArrayList<>();

    public Field2d() {
        FieldObject2d obj = new FieldObject2d("Robot");
        obj.setPose(new Pose2d(0, 0, 0));
        objects.add(obj);
    }

    /**
     * Set the robot pose from a Pose object.
     *
     * @param pose 2D pose
     */
    public synchronized void setRobotPose(Pose2d pose) {
        objects.get(0).setPose(pose);
    }

    /**
     * Set the robot pose from x, y, and rotation.
     *
     * @param xMeters X location, in meters
     * @param yMeters Y location, in meters
     * @param rotation rotation
     */
    public synchronized void setRobotPose(double xMeters, double yMeters, Rotation2d rotation) {
        objects.get(0).setPose(xMeters, yMeters, rotation);
    }

    /**
     * Get the robot pose.
     *
     * @return 2D pose
     */
    public synchronized Pose2d getRobotPose() {
        return objects.get(0).getPose();
    }

    /**
     * Get or create a field object.
     *
     * @param name The field object's name.
     * @return Field object
     */
    public synchronized FieldObject2d getObject(String name) {
        for (FieldObject2d obj : objects) {
            if (obj.name.equals(name)) {
                return obj;
            }
        }
        FieldObject2d obj = new FieldObject2d(name);
        objects.add(obj);
        return obj;
    }

    /**
     * Get the robot object.
     *
     * @return Field object for robot
     */
    public synchronized FieldObject2d getRobotObject() {
        return objects.get(0);
    }
}
