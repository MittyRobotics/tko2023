package com.github.mittyrobotics.autonomous.pathfollowing.math;

public class Point {
    private double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(Vector vector) {
        this(vector.getX(), vector.getY());
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public static Point multiply(double scalar, Point point) {
        return new Point(scalar * point.getX(), scalar * point.getY());
    }

    public static Point add(Point v1, Point v2) {
        return new Point(v1.getX() + v2.getX(), v1.getY() + v2.getY());
    }

    public static double getDistance(Point point1, Point point2) {
        return Math.sqrt(
                (point2.getX() - point1.getX()) * (point2.getX() - point1.getX()) +
                        (point2.getY() - point1.getY()) * (point2.getY() - point1.getY())
        );
    }

    @Override
    public String toString() {
        return String.format("(%f, %f)", getX(), getY());
    }
}
