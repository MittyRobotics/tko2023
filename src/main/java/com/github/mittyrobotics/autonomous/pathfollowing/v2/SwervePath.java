package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.pathfollowing.math.*;

public class SwervePath {
    public QuinticHermiteSpline spline;
    public double maxvel, maxaccel, maxdecel, startvel, endvel, lookahead;

    public SwervePath(QuinticHermiteSpline spline, double lookahead, double maxvel, double maxaccel, double maxdecel, double startvel, double endvel) {
        this.spline = spline;
        this.maxaccel = maxaccel;
        this.maxdecel = maxdecel;
        this.maxvel = maxvel;
        this.startvel = startvel;
        this.endvel = endvel;
        this.lookahead = lookahead;
    }

    public Vector updateLinear(Pose robot, double dt, double curvel) {
        double vel = Math.min(maxvel, curvel + dt * maxaccel);

        double closestT = spline.getClosestPoint(robot, 50, 5);
        double lengthToClosest = spline.getLength(closestT, 17);

        double distanceToEnd = spline.getLength() - lengthToClosest;
        double lookaheadDist = lengthToClosest + lookahead;

        Point lookahead = getLookahead(lookaheadDist);

        System.out.println(lookahead);

        vel = Math.min(getMaxVelToEnd(distanceToEnd), vel);

        double angleToLookahead = new Vector(robot.getPosition(), lookahead).getAngle().getRadians()
                - robot.getHeading().getRadians();

        return new Vector(new Angle(angleToLookahead), vel);
    }

    public Point getGoal() {
        return spline.getEnd();
    }

    public Point getLookahead(double dist) {
        if(dist < spline.getLength()) return spline.get(spline.getTFromLength(dist));
        else {
            double remainder = dist - spline.getLength();
            double endAngle = spline.getEndVel().getAngle().getRadians();
            Point endPose = spline.getEnd();

            return Point.add(endPose, new Point(Math.cos(endAngle) * remainder, Math.sin(endAngle) * remainder));
        }
    }

    public double getMaxVelToEnd(double distToEnd) {
        return Math.sqrt(endvel * endvel + 2 * maxdecel * distToEnd);
    }

    public boolean isFinished(Pose p, double linearThreshold) {
        return new Vector(p.getPosition(), spline.getEnd()).getMagnitude() <= linearThreshold;
    }
}
