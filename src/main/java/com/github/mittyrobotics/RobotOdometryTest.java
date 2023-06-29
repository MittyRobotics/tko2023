package com.github.mittyrobotics;

import com.github.mittyrobotics.arm.StateMachine;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.util.math.Pose;
import edu.wpi.first.wpilibj.TimedRobot;

public class RobotOdometryTest extends TimedRobot {

    @Override
    public void robotInit() {
        Odometry.initOdometry(new Pose(0, 0, 0));
    }

    @Override
    public void robotPeriodic() {
        Odometry.updateFromLimelight();
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
