package com.github.mittyrobotics.autonomous;



import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
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

    public Limelight() {

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
//        limelightTable.getEntry("pipeline").setValue(2);
        hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
        limelightPose =
                limelightTable.getEntry("botpose").getDoubleArray(new double[] {0., 0., 0., 0., 0., 0., 0.});
        targetDist = limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[] {0., 0., 0., 0., 0., 0.});

//        System.out.println("TARGET: " + hasTarget + " at pipeline " + limelightTable.getEntry("pipeline").getDouble(-6) + "\n\n\n");
        if (hasTarget) {
            //x, y, theta

            Pose tempPose = new Pose(new Point(limelightPose[0] * 39.37 + 325.61, limelightPose[1] * 39.37 + 157.863), new Angle(limelightPose[5] * Math.PI/180.));
            if (Gyro.getInstance().getAngleOffset() == null && SwerveSubsystem.getInstance().getDesiredVel().getMagnitude() < 0.1) {
//                Gyro.getInstance().setAngleOffset(tempPose.getHeading().getRadians() - Gyro.getInstance().getHeadingRadiansNoOffset());
//                System.out.println("This ran");
            }
            setPose(tempPose);
            latency = limelightPose[6];
            double tempDist = targetDist[2];
            setDist(tempDist);
            // z: 10.401, x: 8.095, y: 0
            //z offset: 1.875
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
