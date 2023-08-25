package com.github.mittyrobotics.util.math;

import static java.lang.Math.PI;

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

    public static double standardize(double angle) {
        return ((angle % (2 * PI)) + 2 * PI) % (2 * PI);
    }

    public static double getRealAngleDistance(double current, double target, boolean cw) {
        switch (getQuadrant(current)) {
            case 1:
                if (cw) return target - current;
                else {
                    if (getQuadrant(target) == 1) return current - target;
                    else return (2 * PI - target) + current;
                }
            case 4:
                if (cw) return target - current;
                else {
                    if (getQuadrant(target) == 1 || getQuadrant(target) == 4) return current - target;
                    else return (2 * PI - target) + current;
                }
            case 3:
                if (!cw) return current - target;
                else {
                    if (getQuadrant(target) == 2 || getQuadrant(target) == 3) return target - current;
                    else return (2 * PI - current) + target;
                }
            case 2:
                if (!cw) return current - target;
                else {
                    if (getQuadrant(target) == 2) return target - current;
                    else return (2 * PI - current) + target;
                }
        }
        return 0;
    }

    public static int getQuadrant(double angle) {
        angle = Angle.standardize(angle);
        if (angle >= 0 && angle < PI / 2) return 1;
        if (angle >= PI / 2 && angle < PI) return 4;
        if (angle >= PI && angle < 3 * PI / 2) return 3;
        if (angle >= 3 * PI / 2 && angle < 2 * PI) return 2;
        return -6;
    }

    @Override
    public String toString() {
        return "(" + radians + " radians" + ")";
    }
}
