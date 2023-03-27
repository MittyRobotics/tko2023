package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand(int tag, int index, double maxvel, double maxaccel, double maxdecel, double startvel, double endvel) {
        super();

        addRequirements(SwerveSubsystem.getInstance());

        boolean left = tag > 4;

        Pose init = Odometry.getInstance().getState();
        // 0 is left from driver perspective
        Pose target = Odometry.getInstance().getScoringZone(tag)[left ? index : 2 - index];
        Pose score = new Pose(Point.add(target.getPosition(), new Point(left ? 32 : -32, 0)), target.getHeading());
        Pose before_score = new Pose(Point.add(score.getPosition(), new Point(left ? 6 : -6, 0)), score.getHeading());

        double dist = Math.abs(init.getPosition().getX() - target.getPosition().getX());
        double extension = Math.max(40, 1 * dist);

        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                init.getPosition(),
                new Vector(dist < 12 ? (left ? 30 : -30) : 0, 0),
                new Vector(0, 0),
                before_score.getPosition(),
                new Vector(left ? -extension : extension, 0),
                new Vector(0, 0)
        );

        QuinticHermiteSpline spline2 = new QuinticHermiteSpline(
                before_score,
                score
        );

        //TODO TWO PATHS??? TINY LOOKAHEAD SECOND
        addCommands(
                new PathFollowingCommand(
                        new SwervePath(spline, 4,
                                maxvel, maxaccel, maxdecel,
                                startvel, maxvel, true
                        ),
                        left ? Math.PI : 0, 3, 0.02,
                        0, 1, 3, 0, 0, false
                ),
                new PathFollowingCommand(
                        new SwervePath(spline2, 2,
                                maxvel, maxaccel, maxdecel,
                                maxvel, endvel, true
                        ),
                        left ? Math.PI : 0, 3, 0.02,
                        0, 1, 3, 0, 0, false
                )
        );
    }
}
