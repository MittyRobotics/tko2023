package com.github.mittyrobotics;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;



public class LimelightTest extends TimedRobot {
    NetworkTable table;
    NetworkTableEntry tx, ty, ta, tv;

    @Override
    public void robotInit() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("ledMode").setValue(0);
        table.getEntry("camMode").setValue(0);
        table.getEntry("pipeline").setValue(0);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
//        limeLight = NetworkTableInstance.getDefault().getTable("limelight");

    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void robotPeriodic() {
        System.out.println(tv.getDouble(-1));
        System.out.println(tx.getDouble(-1));
        System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(-1000) == 1);

//        System.out.println(table.getEntry("pipeline").getDouble(-100000));

//        double x = tx.getDouble(0.0);
//        double y = ty.getDouble(0.0);
//        double area = ta.getDouble(0.0);
//        System.out.println("x: " + x);
//        System.out.println("y: " + y);
//        System.out.println("a: " + area);

//        System.out.println(table.getEntry("tx"));
//        SmartDashboard.putNumber("LimelightY", y);
//        SmartDashboard.putNumber("LimelightArea", area);

//        System.out.println(limeLight.getEntry("ta").getDouble(5.0));
//        System.out.println(limeLight.getInstance().isConnected());

    }

    @Override
    public void autonomousPeriodic() {
        super.autonomousPeriodic();
    }

    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
    }
}
