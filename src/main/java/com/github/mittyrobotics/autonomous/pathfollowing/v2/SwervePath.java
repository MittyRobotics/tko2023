package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;

import java.util.logging.Logger;

public class SwervePath {
    public QuinticHermiteSpline spline;
    public double maxvel, maxaccel, maxdecel, startvel, endvel, lookahead, vel, closestT, lengthToClosest;
    public boolean auto;

    public SwervePath(QuinticHermiteSpline spline, double lookahead, double maxvel, double maxaccel, double maxdecel, double startvel, double endvel, boolean auto, boolean meters) {
        double factor = meters ? 39.37 : 1;

        this.spline = spline;
        this.maxaccel = maxaccel * factor;
        this.maxdecel = maxdecel * factor;
        this.maxvel = maxvel * factor;
        this.startvel = startvel * factor;
        this.endvel = endvel * factor;
        this.lookahead = lookahead;
        this.auto = auto;

        vel = 0;
        closestT = 0;
        lengthToClosest = 0;
    }

    public SwervePath(QuinticHermiteSpline spline, double lookahead, double maxvel, double maxaccel, double maxdecel, double startvel, double endvel, boolean auto) {
        this(spline, lookahead, maxvel, maxaccel, maxdecel, startvel, endvel, auto, true);
    }

    public Vector updateLinear(Pose robot, double dt) {
        double closestT = spline.getClosestPoint(robot, 50, 5);
        System.out.println("CLOSEST T: " + closestT);
//        LoggerInterface.getInstance().put("CLOSET", closestT);
        lengthToClosest = spline.getLength(closestT, 17);

//        System.out.println("LENGTH TO CLOSEST " + lengthToClosest);

        double distanceToEnd = spline.getLength() - lengthToClosest;
        double lookaheadDist = lengthToClosest + lookahead;

        Point lookahead = getLookahead(lookaheadDist);
        System.out.println("LOOKAHEAD: " + lookahead);

        if(auto) {
            vel = Math.min(maxvel, vel + dt * maxaccel);
            System.out.println("AUTO VEL: " + vel);
            vel = Math.min(getMaxVelToEnd(distanceToEnd), vel);

        } else {
            double leftY = OI.getInstance().getDriveController().getLeftX();
            double leftX = -OI.getInstance().getDriveController().getLeftY();
            vel = Math.sqrt(leftY * leftY + leftX * leftX) * SwerveConstants.MAX_LINEAR_SPEED_INCHES_PER_SECOND;
        }

        System.out.println("VECTOR TO LOOKAHEAD " + new Vector(robot.getPosition(), lookahead));

//        double angleToLookahead = new Vector(robot.getPosition(), lookahead).getAngle().getRadians();

//        System.out.println("Lookahead " + lookahead + "    " + "Robot " + robot + "   Angle to " + angleToLookahead);

//        return new Vector(new Angle(angleToLookahead), vel);
//        Vector vectorToLookahead = new Vector(robot.getPosition(), lookahead);
        Vector vectorToLookahead = spline.getVelocityVector(closestT);
        return Vector.multiply(vel / vectorToLookahead.getMagnitude(), vectorToLookahead);
    }

    public void changeSpline(QuinticHermiteSpline nSpline) {
        spline = nSpline;
    }

    public double getVel() {
        return vel;
    }

    public double getHeadingGoal(double startHeading, double endHeading, double angStart, double angEnd) {
        double fraction = lengthToClosest / spline.getLength();
        System.out.println("FRACTION: " + fraction);
        if (fraction >= angStart) {
            return doSigmoidInterpolation(startHeading, endHeading, (fraction - angStart) / (angEnd - angStart));
        } else return startHeading;
    }

    private static double sigmoid(double x, double a) {
        return 1. / (1 + Math.exp(-a * (x - 0.5)));
    }

    public static double doSigmoidInterpolation(double start, double end, double t) {
//        LoggerInterface.getInstance().put("SIG T", t);
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
        return Math.sqrt(endvel * endvel + 2 * maxdecel * distToEnd);
    }

    public boolean isFinished(Pose p, double linearThreshold) {
        return new Vector(p.getPosition(), spline.getEnd()).getMagnitude() <= linearThreshold;
    }
}
