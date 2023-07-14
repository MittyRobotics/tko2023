package com.github.mittyrobotics.autonomous;

import com.github.mittyrobotics.util.math.Point;
import com.github.mittyrobotics.util.math.Pose;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Odometry {
    private static NetworkTable limelightTable;
    private static Pose curPose;
    private static double[] limelightPose, targetDist;
    private static boolean hasTarget;

    private static double zDistToTarget;

    public Odometry() {

    }

    public static void initOdometry(Pose startPose, double startDist) {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        curPose = startPose;
        zDistToTarget = startDist;
        limelightTable.getEntry("ledMode").setValue(0);
        limelightTable.getEntry("camMode").setValue(0);
        limelightTable.getEntry("pipeline").setValue(2);
    }

    public static void updateFromLimelight() {
        //x = long side
        //y = short side
        hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
        limelightPose =
                limelightTable.getEntry("botpose").getDoubleArray(new double[] {0., 0., 0., 0., 0., 0., 0.});
        targetDist = limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[] {0., 0., 0., 0., 0., 0.});


        if (hasTarget) {
            //x, y, theta
            Pose tempPose = new Pose(limelightPose[0], limelightPose[1], limelightPose[5], false);
            setPose(tempPose);
            double tempDist = targetDist[2];
            setDist(tempDist);
        }

    }

    public static double getZDistToTarget() {
        return zDistToTarget;
    }

    public static void setPose(Pose p) {
        curPose = p;
    }

    public static void setDist(double d) {
        zDistToTarget = d;
    }

    public static Pose getPose() {
        return curPose;
    }
}
