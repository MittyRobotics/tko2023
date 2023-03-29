package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TeleopScoreCommand extends SequentialCommandGroup {


    public TeleopScoreCommand(int tag, int index) {
        super();

        System.out.println("\n\n\n\n\n\n\n\nFUCK\n\n\n");
        boolean left = tag > 4;

        Pose init = Odometry.getInstance().getState();
        // 0 is left from driver perspective
        Pose target = Odometry.getInstance().getScoringZone(tag)[left ? index : 2 - index];
        Pose score = new Pose(Point.add(target.getPosition(), new Point(left ? 32 : -32, 0)),
                new Angle(left ? Math.PI : 0));
        Pose before_score = new Pose(Point.add(score.getPosition(), new Point(left ? 10 : -10, 0)), score.getHeading());

        double dist = Math.abs(init.getPosition().getX() - target.getPosition().getX());
        double extension = Math.max(20, 0.5 * dist);

        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                init.getPosition(),
                new Vector(dist < 12 ? (left ? 30 : -30) : 0, 0),
                new Vector(0, 0),
                before_score.getPosition(),
                new Vector(left ? -extension : extension, 0),
                new Vector(0, 0)
        );

        QuinticHermiteSpline spline2 = new QuinticHermiteSpline(
                before_score.getPosition(),
                score.getPosition()
        );

        addCommands(
                new InstantCommand(() -> {

                }),
            new PathFollowingCommand(
                    new SwervePath(spline, 4,
                            0, 0, 0,
                            0, 0, false
                    ),
                    left ? Math.PI : 0, 3, 0.02,
                    0, 1, 3, 0, 0, false
            ), new PathFollowingCommand(
                    new SwervePath(spline2, 2,
                            0, 0, 0,
                            0, 0, false
                    ),
                    left ? Math.PI : 0, 3, 0.02,
                    0, 1, 3, 0, 0, false
            )
        );
    }
}
