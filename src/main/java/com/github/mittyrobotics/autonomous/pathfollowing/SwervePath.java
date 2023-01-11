package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwervePath {
    private QuinticHermiteSpline spline;
    private Angle startHeading, endHeading;
    private double initSpeed, endSpeed, maxSpeed, accel, decel, minAngular, lookahead, kp, ki, kd, whenToEnd;

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
    }

    public SwervePath(QuinticHermiteSpline function, Angle startHeading, Angle endHeading) {
        this.spline = function;
        this.startHeading = startHeading;
        this.endHeading = endHeading;

    }

    public Pose getByT(double t) {
        return new Pose(
                spline.get(t),
                Pose.doSigmoidInterpolation(startHeading, endHeading, t)
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

    public Vector getVectorToLookahead(Pose robot, double lookahead) {
        // TODO: 9/2/2022 Fix lookahead format - should be resolved
        double t = getTForLookahead(robot, lookahead);

        Point splinePoint = getByT(t).getPosition();

        SmartDashboard.putNumber("t for lookahead", t);
        SmartDashboard.putString("point for lookahead", splinePoint.toString());

        return new Vector(robot.getPosition(), splinePoint);
    }

    public Angle getHeadingAtLookahead(Pose robot, double lookahead) {
        double t = getTForLookahead(robot, lookahead);
        double length = spline.getLength(t, 17);
        double total = spline.getLength(whenToEnd, 17);
        double fraction = length/total;
//        double angle = fraction * (endHeading.getRadians() - startHeading.getRadians()) + startHeading.getRadians();

//        return new Angle(angle);
//        return getByT(t).getHeading();
        return Pose.doSigmoidInterpolation(startHeading, endHeading, fraction);
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
}
