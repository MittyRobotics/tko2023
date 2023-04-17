package com.github.mittyrobotics.autonomous.pathfollowing.math;

public class Line {
    private double slope, yIntercept;

    public Line(double slope, double yIntercept) {
        this.slope = slope;
        this.yIntercept = yIntercept;
    }

    public Line(double slope, Point point) {
        this.slope = slope;
        this.yIntercept = -1 * slope * point.getX() + point.getY();
    }

    public Line(Angle angle, Point point) {
        this(Math.tan(angle.getRadians()), point);
    }

    public Line(Point point1, Point point2) {
        this((point2.getY() - point1.getY())/(point2.getX() - point1.getX()), point1);
    }

    public Line getTangentAtPoint(Point other) {
        return new Line(-1./slope, other);
    }

    public double y(double x) {
        return slope * x + yIntercept;
    }

    public static Point getIntersection(Line line1, Line line2) {
        double x = (line2.yIntercept - line1.yIntercept) / (line1.slope - line2.slope);
        return new Point(x, line1.y(x));
    }

    public double getSlope() {
        return slope;
    }
}
