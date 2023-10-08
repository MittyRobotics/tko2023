package frc.robot.subsystems;

import frc.robot.util.math.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable limelightTable;
    private Pose curPose;
    private int tagID = -1;
    private double latency;
    private double[] limelightPose, targetDist;
    private boolean hasTarget;

//    private boolean checkingGyro = false;

    private double zDistToTarget;

    public Limelight() {
        // z: 10.401, x: 8.095, y: 0
        //z offset: 1.875
    }

    public void initLL(Pose startPose, double startDist) {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        curPose = startPose;
        zDistToTarget = startDist;
        limelightTable.getEntry("ledMode").setValue(0);
        limelightTable.getEntry("camMode").setValue(0);
        limelightTable.getEntry("pipeline").setValue(2);
    }

    public void updatePose() {
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
//            System.out.println("LL angle " + limelightPose[5]);
//            if (Gyro.getInstance().getAngleOffset() == null && checkingGyro) {
//                Gyro.getInstance().setAngleOffset(Math.PI + tempPose.getHeading().getRadians() - Gyro.getInstance().getHeadingRadiansNoOffset(), true);
//            }
            setPose(tempPose);
            latency = limelightPose[6];
            double tempDist = targetDist[2];
            setDist(tempDist);
        } else setPose(null);

    }

    public void updateClosestTag() {
        hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
        if (hasTarget) tagID = (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    public double getXDistToTarget() {
        return zDistToTarget * 39.37;
    }

    public void setPose(Pose p) {
        curPose = p;
    }

    public Pose getPose() {
        return curPose;
    }

    public void setDist(double d) {
        zDistToTarget = d;
    }

    public double getLatency() {
        return latency * 1000000;
    }

    public int getClosestTag() {
        if (tagID == -1) return 1;
        return tagID;
    }

//    public void setCheckingGyro(boolean checkingGyro) {
//        this.checkingGyro = checkingGyro;
//    }

//    public void setAngleOffset() {
//        hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
//        limelightPose =
//                limelightTable.getEntry("botpose").getDoubleArray(new double[] {0., 0., 0., 0., 0., 0., 0.});
//        if (hasTarget) {
//            if (Gyro.getInstance().getAngleOffset() == null && checkingGyro) {
//                Gyro.getInstance().setAngleOffset(new Angle(limelightPose[5], false).getRadians() - Gyro.getInstance().getHeadingRadiansNoOffset(), true);
//            }
//        } else {
//            if (Gyro.getInstance().getAngleOffset() == null && checkingGyro) {
//                Gyro.getInstance().setAngleOffset((Odometry.getInstance().FIELD_LEFT_SIDE ? Math.PI : 0), true);
//            }
//        }
//    }
}
