package com.github.mittyrobotics.util.math;

public class Vector {
    private double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector(Angle angle, double magnitude) {
        this(magnitude * Math.cos(angle.getRadians()), magnitude * Math.sin(angle.getRadians()));
    }

    public Vector(Point p) {
        this(p.getX(), p.getY());
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public Angle getAngle() {
        return new Angle(Math.atan2(y, x), true);
    }

    public static Vector add(Vector v1, Vector v2) {
        return new Vector(v1.x + v2.x, v1.y + v2.y);
    }

    public static Vector multiply(double scalar, Vector v) {
        return new Vector(scalar * v.x, scalar * v.y);
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
