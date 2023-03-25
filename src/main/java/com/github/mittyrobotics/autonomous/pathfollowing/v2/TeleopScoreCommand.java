package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TeleopScoreCommand extends SequentialCommandGroup {
    public TeleopScoreCommand(int tag, int index) {
        super();

        boolean left = tag > 4;

        Pose init = Odometry.getInstance().getState();
        // 0 is left from driver perspective
        Pose target = Odometry.getInstance().getScoringZone(tag)[left ? index : 2 - index];
        Pose score = new Pose(Point.add(target.getPosition(), new Point(left ? 32 : -32, 0)), target.getHeading());

        double extension = Math.max(60, 1 * Math.abs(init.getPosition().getX() - target.getPosition().getX()));

        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                init.getPosition(),
                new Vector(0, 0),
                new Vector(0, 0),
                score.getPosition(),
                new Vector(left ? -extension : extension, 0),
                new Vector(0, 0)
        );

        addCommands(
            new PathFollowingCommand(
                    new SwervePath(spline, 4,
                            0, 0, 0,
                            0, 0, false
                    ),
                    left ? Math.PI : 0, 3, 0.02,
                    0, 0.75, 2.5, 0, 0, false
            )
        );
    }
}
