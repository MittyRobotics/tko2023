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

    public double y(double x) {
        return slope * x + yIntercept;
    }

    public static Point getIntersection(Line line1, Line line2) {
        double x = (line2.yIntercept - line1.yIntercept) / (line1.slope - line2.slope);
        return new Point(x, line1.y(x));
    }
}
