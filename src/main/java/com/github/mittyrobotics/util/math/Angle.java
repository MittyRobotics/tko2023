package com.github.mittyrobotics.util.math;

public class Angle {
    private double radians;

    public Angle(double angle, boolean radians) {
        this.radians = angle * (radians ? 1 : Math.PI / 180);
    }

    public double getRadians() {
        return radians;
    }

    public double getDegrees() {
        return radians * 180 / Math.PI;
    }

    public static Angle add(Angle a1, Angle a2) {
        return new Angle(a1.radians + a2.radians, true);
    }

    @Override
    public String toString() {
        return "(" + radians + " radians" + ")";
    }
}
