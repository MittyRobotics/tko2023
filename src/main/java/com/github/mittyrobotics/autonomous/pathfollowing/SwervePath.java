package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.util.math.*;
import com.github.mittyrobotics.util.math.autonomous.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwervePath {
    private QuinticHermiteSpline spline;
    private Angle startHeading, endHeading;
    private double initSpeed, endSpeed, maxSpeed, accel, decel, minAngular, kp, ki, kd, angStart, angEnd, total;
    private double correctionScale, tangentScale;

    public SwervePath(QuinticHermiteSpline spline, Angle startHeading, Angle endHeading,
                      double initSpeed, double endSpeed, double maxSpeed, double accel, double decel, double minAngular,
                      double angStart, double angEng, double correctionScale, double tangentScale,
                      double kp, double ki, double kd) {
        this.spline = spline;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
        this.initSpeed = initSpeed;
        this.endSpeed = endSpeed;
        this.maxSpeed = maxSpeed;
        this.accel = accel;
        this.decel = decel;
        this.minAngular = minAngular;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.angStart = angStart;
        this.angEnd = angEng;
        this.correctionScale = correctionScale;
        this.tangentScale = tangentScale;

        total = spline.getLength(1.0, 17);
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

    public Vector getErrorVector(Pose robot) {
        double closetT = spline.getClosestPoint(robot, 50, 7);
        Point splinePoint = getByT(closetT).getPoint();

        return new Vector(robot.getPoint(), splinePoint);
    }

    public Angle getCurrentDesiredHeading(double length) {
        double lengthStart = spline.getLength(0, angStart, 17);
        double lengthEnd = spline.getLength(0, angEnd, 17);
        if (length < lengthStart) return startHeading;
        if (length > lengthEnd) return endHeading;
        double fraction = (length - lengthStart) / (lengthEnd - lengthStart);

        return doSigmoidInterpolation(startHeading, endHeading, fraction);
    }

    private static double sigmoid(double x, double a) {
        return 1. / (1 + Math.exp(-a * (x - 0.5)));
    }

    public static Angle doSigmoidInterpolation(Angle start, Angle end, double t) {
        return new Angle(
                start.getRadians() + (end.getRadians() - start.getRadians()) * sigmoid(t, 10), true
        );
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

    public double getCorrectionScale() {
        return correctionScale;
    }

    public double getTangentScale() {
        return tangentScale;
    }
}
