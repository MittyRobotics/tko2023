package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.*;

public class AutoPathManager {
    public static QuinticHermiteSpline scoreSpline1, scoreSpline2, scoreSpline3;
    public static boolean left;

    public static void updateSplines(int tag, int index) {

        Pose init = Odometry.getInstance().getState();

        if (tag == -1)
            tag = Odometry.getClosestScoringZone(init);

        left = tag > 4;

        // 0 is left from driver perspective
        Pose target = Odometry.getInstance().getScoringZone(tag)[left ? index : 2 - index];
        Pose score = new Pose(Point.add(target.getPosition(), new Point(left ? 32 : -32, 0)),
                new Angle(left ? Math.PI : 0));
        Pose before_score = new Pose(Point.add(score.getPosition(), new Point(left ? 10 : -10, 0)), score.getHeading());

        double dist = Math.abs(init.getPosition().getX() - target.getPosition().getX());
        double extension = Math.max(40, 1 * dist);

        scoreSpline1 = new QuinticHermiteSpline(
                init.getPosition(),
                new Vector(dist < 12 ? (left ? 30 : -30) : 0, 0),
                new Vector(0, 0),
                before_score.getPosition(),
                new Vector(left ? -extension : extension, 0),
                new Vector(0, 0)
        );

        scoreSpline2 = new QuinticHermiteSpline(
                before_score.getPosition(),
                score.getPosition()
        );

        scoreSpline3 = new QuinticHermiteSpline(
                init.getPosition(),
                new Vector(dist < 12 ? (left ? 30 : -30) : 0, 0),
                new Vector(0, 0),
                score.getPosition(),
                new Vector(left ? -extension : extension, 0),
                new Vector(0, 0)
        );
    }

}
