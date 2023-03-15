package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwervePath {
    private QuinticHermiteSpline spline;
    private Angle startHeading, endHeading;
    private double initSpeed, endSpeed, maxSpeed, accel, decel, minAngular, lookahead, kp, ki, kd, whenToEnd, total;

    public SwervePath(QuinticHermiteSpline spline, Angle startHeading, Angle endHeading, double initSpeed, double endSpeed, double maxSpeed, double accel, double decel, double minAngular, double lookahead, double kp, double ki, double kd, double whenToEnd) {
        this.spline = spline;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
        this.initSpeed = initSpeed;
        this.endSpeed = endSpeed;
        this.maxSpeed = maxSpeed;
        this.accel = accel;
        this.decel = decel;
        this.minAngular = minAngular;
        this.lookahead = lookahead;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.whenToEnd = whenToEnd;

        total = spline.getLength(whenToEnd, 17);
    }

    public SwervePath(QuinticHermiteSpline function, Angle startHeading, Angle endHeading) {
        this.spline = function;
        this.startHeading = startHeading;
        this.endHeading = endHeading;

    }

    public Pose getByT(double t) {
        return new Pose(
                spline.get(t),
//                Pose.doSigmoidInterpolation(startHeading, endHeading, t)
                spline.getVelocityVector(t).getAngle()
        );
    }

    public QuinticHermiteSpline getSpline() {
        return spline;
    }

    public double getTForLookahead(Pose robot, double lookahead) {
        double closestPoint = spline.getClosestPoint(robot, 50, 10);
        double length = spline.getLength(closestPoint, 17);
        length += lookahead;

        return spline.getTFromLength(length);
    }

    public Point getLookaheadPoint(Pose robot, double lookahead) {
        return spline.get(getTForLookahead(robot, lookahead));
    }

    public Vector getVectorToLookahead(Pose robot, double length, double lookahead) {
        // TODO: 9/2/2022 Fix lookahead format - should be resolved
        double t = spline.getTFromLength(length + lookahead);

        Point splinePoint = getByT(t).getPosition();

        SmartDashboard.putNumber("t for lookahead", t);
        SmartDashboard.putString("point for lookahead", splinePoint.toString());

        return new Vector(robot.getPosition(), splinePoint);
    }

    public Angle getCurrentDesiredHeading(double length) {
        double fraction = length/total;

        return Pose.doSigmoidInterpolation(startHeading, endHeading, fraction);
    }

    public Angle getEndHeading() {
        return endHeading;
    }

    public double getCurvature(double t) {
        return spline.curvature(t);
    }

    public double getInitSpeed() {
        return initSpeed;
    }

    public double getEndSpeed() {
        return endSpeed;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getAccel() {
        return accel;
    }

    public double getDecel() {
        return decel;
    }

    public double getMinAngular() {
        return minAngular;
    }

    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

    public double getLookahead() {
        return lookahead;
    }
}
