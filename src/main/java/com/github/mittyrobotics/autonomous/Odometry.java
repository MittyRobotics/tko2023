package com.github.mittyrobotics.autonomous;

import com.github.mittyrobotics.util.math.Point;
import com.github.mittyrobotics.util.math.Pose;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Odometry {
    private NetworkTable limelightTable;
    private Pose curPose;
    private double[] limelightPose;
    private boolean hasTarget;

    public Odometry() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        curPose = new Pose(0, 0, 0);
    }

    public void initOdometry(Pose startPose) {
        curPose = startPose;
    }

    public void updateFromLimelight() {
        //x = long side
        //y = short side
        hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
        limelightPose =
                limelightTable.getEntry("botpose").getDoubleArray(new double[] {0., 0., 0., 0., 0., 0.});

        if (hasTarget) {
            //x, y, theta
            Pose tempPose = new Pose(limelightPose[0], limelightPose[1], limelightPose[5]);
            setPose(tempPose);
        }

    }

    public void setPose(Pose p) {
        curPose = p;
    }

    public Pose getPose() {
        return curPose;
    }
}
