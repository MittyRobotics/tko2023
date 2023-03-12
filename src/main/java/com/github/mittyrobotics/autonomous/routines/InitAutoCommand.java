package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class InitAutoCommand extends InstantCommand {
    public InitAutoCommand(Pose apporxStartPose) {
        super(() -> {
            SwerveSubsystem.getInstance().setPose(apporxStartPose);
            Odometry.getInstance().setState(
                    apporxStartPose.getPosition().getX(),
                    apporxStartPose.getPosition().getY(),
                    apporxStartPose.getHeading().getRadians()
            );
        });
    }
}
