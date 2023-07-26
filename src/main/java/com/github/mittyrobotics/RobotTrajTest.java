package com.github.mittyrobotics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;

import java.util.List;

public class RobotTrajTest extends TimedRobot {
    private Trajectory traj;
    @Override
    public void robotInit() {

        // missing config
        traj = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(0, 2)),
                new Pose2d(3, 3, new Rotation2d(0))
        );
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
    }
}
