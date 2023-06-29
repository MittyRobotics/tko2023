package com.github.mittyrobotics.util.math;

public class Pose {
    private Point point;
    private Angle angle;

    public Pose(Point p, Angle a) {
        point = p;
        angle = a;
    }

    public Pose(double x, double y, double theta) {
        point = new Point(x, y);
        angle = new Angle(theta, true);
    }

    public Point getPoint() {
        return point;
    }

    public Angle getAngle() {
        return angle;
    }

    @Override
    public String toString() {
        return "{" + point.toString() + ", " + angle.toString() + "}";
    }
}
