package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OldAutoScoreCommand extends SequentialCommandGroup {
    public OldAutoScoreCommand(Pose target, boolean auto) {
        super();
        Pose init = Odometry.getInstance().getState();
        target = new Pose(Point.add(target.getPosition(), new Point(Odometry.getInstance().FIELD_LEFT_SIDE ? 32 : -32, 0)), target.getHeading());
        Pose mid = new Pose(Point.add(target.getPosition(), new Point((Odometry.getInstance().FIELD_LEFT_SIDE ? 1 : -1) * 9, 0)), target.getHeading());
        addCommands(
                new InstantCommand(() -> LedSubsystem.getInstance().setAltColor(LedSubsystem.Color.BLUE)),
                new AutoDriveScoreCommand(1, 0.02, true,
                        new OldSwervePath(new QuinticHermiteSpline(init, mid),
                                init.getHeading(), mid.getHeading(),
                                0, 1, 1.5, 3, 1.5,
                                0, 0, 3.5, 0, 0.02, 0.5
                        ), auto
                ),
                new AutoDriveScoreCommand(1.5, 0.02, false,
                        new OldSwervePath(new QuinticHermiteSpline(mid, target),
                                mid.getHeading(), target.getHeading(),
                                0, 0, 1, 1.5, 3,
                                0, 0, 3, 0, 0.02, 0.5
                        ), auto
                ),
                new InstantCommand(() -> LedSubsystem.getInstance().disableDriveAltColor())
        );
    }
}
