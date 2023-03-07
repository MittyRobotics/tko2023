package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveAutoScoreCommand extends SequentialCommandGroup {
    public SwerveAutoScoreCommand(Pose target) {
        super();
        Pose init = Odometry.getInstance().getState();
        Pose mid = new Pose(Point.add(target.getPosition(), new Point((Odometry.getInstance().FIELD_LEFT_SIDE ? 1 : -1) * 10, 0)), target.getHeading());
        Pose end = target;
        addCommands(
                new SwervePurePursuitCommand(2, 0.05,
                        new SwervePath(new QuinticHermiteSpline(init, mid),
                                init.getHeading(), mid.getHeading(),
                                0, 0, 6., 8., 3, 0.2, 0.4, 2.5, 0, 0.02, 0.5)),
                new SwerveAutoDriveToTargetCommand(0.05, 0.05,
                        new SwervePath(new QuinticHermiteSpline(mid, end),
                                mid.getHeading(), end.getHeading(),
                                0, 0, 6., 8., 3, 0.2, 0.4, 2.5, 0, 0.02, 0.5))
        );
    }
}
