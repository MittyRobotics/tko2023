package com.github.mittyrobotics.autonomous.pathfollowing.math;

public class Vector {
    private double x, y, magnitude;
    private Angle angle;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;

        setAngle(new Angle(Math.atan2(y, x)));
        setMagnitude(Math.sqrt(x * x + y * y));
    }

    public Vector(Angle angle, double magnitude) {
        this.magnitude = magnitude;
        this.angle = angle;

        setX(magnitude * Math.cos(angle.getRadians()));
        setY(magnitude * Math.sin(angle.getRadians()));
    }

    public Vector(Point tail, Point tip) {
        this(
                tip.getX() - tail.getX(),
                tip.getY() - tail.getY()
        );
    }

    public Vector(Point tip) {
        this(
                new Point(0, 0),
                tip
        );
    }

    public static Vector multiply(double scalar, Vector vector) {
        return new Vector(scalar * vector.getX(), scalar * vector.getY());
    }

    public static Vector add(Vector v1, Vector v2) {
        return new Vector(v1.getX() + v2.getX(), v1.getY() + v2.getY());
    }

    public double getX() {
        return x;
    }

    private void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    private void setY(double y) {
        this.y = y;
    }

    public Angle getAngle() {
        return angle;
    }

    private void setAngle(Angle angle) {
        this.angle = angle;
    }

    public double getMagnitude() {
        return magnitude;
    }

    private void setMagnitude(double magnitude) {
        this.magnitude = magnitude;
    }

    public double getSlope() {
        return getY() / getX();
    }

    @Override
    public String toString() {
        return new Point(this).toString();
    }
}
