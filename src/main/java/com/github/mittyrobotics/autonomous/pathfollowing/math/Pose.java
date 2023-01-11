package com.github.mittyrobotics.autonomous.pathfollowing.math;

public class Pose {
    private Point position;
    private Angle heading;

    public Pose(Point position, Angle heading) {
        this.position = position;
        this.heading = heading;
    }

    public static Angle doLinearInterpolation(Angle start, Angle end, double t) {
        return new Angle(
                start.getRadians() + (t - 0) * (end.getRadians() - start.getRadians()) / (1 - 0)
        );
    }

    private static double sigmoid(double x, double a) {
        return 1. / (1 + Math.exp(-a * (x - 0.5)));
    }

    public static Angle doSigmoidInterpolation(Angle start, Angle end, double t) {
        return new Angle(
                start.getRadians() + (end.getRadians() - start.getRadians()) * sigmoid(t, 10)
        );
    }

    public Point getPosition() {
        return position;
    }

    public Angle getHeading() {
        return heading;
    }

    @Override
    public String toString() {
        return position.toString() + " " + heading.toString();
    }
}
