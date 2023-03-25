package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import com.github.mittyrobotics.util.Gyro;

import java.util.logging.Logger;

public class SwervePath {
    public QuinticHermiteSpline spline;
    public double maxvel, maxaccel, maxdecel, startvel, endvel, lookahead, vel, closestT, lengthToClosest;

    public SwervePath(QuinticHermiteSpline spline, double lookahead, double maxvel, double maxaccel, double maxdecel, double startvel, double endvel) {
        this.spline = spline;
        this.maxaccel = maxaccel;
        this.maxdecel = maxdecel;
        this.maxvel = maxvel;
        this.startvel = startvel;
        this.endvel = endvel;
        this.lookahead = lookahead;

        vel = 0;
        closestT = 0;
        lengthToClosest = 0;
    }

    public Vector updateLinear(Pose robot, double dt, double curvel) {
        vel = Math.min(maxvel, vel + dt * maxaccel);

        closestT = spline.getClosestPoint(robot, 100, 5);
        LoggerInterface.getInstance().put("CLOSET", closestT);
        lengthToClosest = spline.getLength(closestT, 17);

        double distanceToEnd = spline.getLength() - lengthToClosest;
        double lookaheadDist = lengthToClosest + lookahead;

        Point lookahead = getLookahead(lookaheadDist);

        vel = Math.min(getMaxVelToEnd(distanceToEnd), vel);

        double angleToLookahead = new Vector(robot.getPosition(), lookahead).getAngle().getRadians()
                - Gyro.getInstance().getHeadingRadians();

//        System.out.println("Lookahead " + lookahead + "    " + "Robot " + robot + "   Angle to " + angleToLookahead);

        return new Vector(new Angle(angleToLookahead), vel);
    }

    public double getHeadingGoal(double startHeading, double endHeading, double angStart, double angEnd) {
        double fraction = lengthToClosest / spline.getLength();
        if (fraction >= angStart) {
            return doSigmoidInterpolation(startHeading, endHeading, (fraction - angStart) / (angEnd - angStart));
        } else return startHeading;
    }

    private static double sigmoid(double x, double a) {
        return 1. / (1 + Math.exp(-a * (x - 0.5)));
    }

    public static double doSigmoidInterpolation(double start, double end, double t) {
        LoggerInterface.getInstance().put("SIG T", t);
        return start + (end - start) * sigmoid(t, 10);
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
        return Math.sqrt(endvel * endvel + 2 * maxdecel * distToEnd / 39.37);
    }

    public boolean isFinished(Pose p, double linearThreshold) {
        return new Vector(p.getPosition(), spline.getEnd()).getMagnitude() <= linearThreshold;
    }
}
