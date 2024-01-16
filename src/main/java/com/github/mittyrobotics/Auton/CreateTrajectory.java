package com.github.mittyrobotics.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.util.List;

public class CreateTrajectory extends SubsystemBase {
    private Trajectory trajectory;
    public CreateTrajectory(double maxVel, double maxAccel, Pose2d poseStart, Pose2d poseEnd, Translation2d... interiorPoints) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        trajectory = TrajectoryGenerator.generateTrajectory(poseStart, List.of(interiorPoints), poseEnd, config);
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

}
