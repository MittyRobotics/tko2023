package com.github.mittyrobotics;

import com.github.mittyrobotics.arm.StateMachine;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.util.math.Pose;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class RobotOdometryTest extends TimedRobot {

//    NetworkTable table;
//    NetworkTableEntry tx, ty, ta, tv;
    @Override
    public void robotInit() {
        Odometry.initOdometry(new Pose(0, 0, 0, false), 0);
//        table = NetworkTableInstance.getDefault().getTable("limelight");
//        table.getEntry("ledMode").setValue(0);
//        table.getEntry("camMode").setValue(0);
//        table.getEntry("pipeline").setValue(2);

    }

    @Override
    public void robotPeriodic() {
//        tx = table.getEntry("tx");
//        System.out.println(tx.getDouble(0));
//        ty = table.getEntry("ty");
//        ta = table.getEntry("ta");
//        tv = table.getEntry("tv");
//        System.out.println(table.getEntry("tx").getDouble(0));
//        System.out.println(table.getEntry("tv").getDouble(0));



        Odometry.updateFromLimelight();
        System.out.println("Z DIST: " + Odometry.getZDistToTarget());
        System.out.println(Odometry.getPose());
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();

    }

    @Override
    public void autonomousPeriodic() {
        super.autonomousPeriodic();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
    }
}
