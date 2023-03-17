package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class InitAutoCommand extends InstantCommand {
    public InitAutoCommand(Pose pose) {
        super(() -> {
            double x = pose.getPosition().getX();
            double y = pose.getPosition().getY();
            double t = pose.getHeading().getRadians();

            SwerveSubsystem.getInstance().resetEncoders();
            Gyro.getInstance().setAngleOffset(t - Gyro.getInstance().getHeadingRadiansNoOffset());
            Odometry.getInstance().setState(x, y, t);
            SwerveSubsystem.getInstance().setPose(new Pose(new Point(0, 0),
                    new Angle(Gyro.getInstance().getHeadingRadians())));
        });
    }
}
