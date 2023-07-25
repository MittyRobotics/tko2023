package com.github.mittyrobotics.autonomous;



import com.github.mittyrobotics.util.math.*;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static NetworkTable limelightTable;
    private static Pose curPose;
    private static double latency;
    private static double[] limelightPose, targetDist;
    private static boolean hasTarget;

    private static double zDistToTarget;

    private Limelight() {
        // z: 10.401, x: 8.095, y: 0
        //z offset: 1.875
    }

    public static void init(Pose startPose, double startDist) {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        curPose = startPose;
        zDistToTarget = startDist;
        limelightTable.getEntry("ledMode").setValue(0);
        limelightTable.getEntry("camMode").setValue(0);
        limelightTable.getEntry("pipeline").setValue(2);
    }

    public static void update() {
        //x = long side
        //y = short side
        hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
        limelightPose =
                limelightTable.getEntry("botpose").getDoubleArray(new double[] {0., 0., 0., 0., 0., 0., 0.});
        targetDist = limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[] {0., 0., 0., 0., 0., 0.});

        System.out.println("TARGET: " + hasTarget + "\n\n\n");
        if (hasTarget) {
            //x, y, theta

            Pose tempPose = new Pose(new Point(limelightPose[0] * 39.37 + 325.61, limelightPose[1] * 39.37 + 157.863), new Angle(limelightPose[5], false));
            if (Gyro.getInstance().getAngleOffset() == null) {
                Gyro.getInstance().setAngleOffset(tempPose.getHeading().getRadians() - Gyro.getInstance().getHeadingRadiansNoOffset());
            }
            setPose(tempPose);
            latency = limelightPose[6];
            double tempDist = targetDist[2];
            setDist(tempDist);
        } else setPose(null);

    }

    public static double getXDistToTarget() {
        return zDistToTarget * 39.37;
    }

    public static void setPose(Pose p) {
        curPose = p;
    }

    public static Pose getPose() {
        return curPose;
    }

    public static void setDist(double d) {
        zDistToTarget = d;
    }

    public static double getLatency() {
        return latency * 1000000;
    }
}
