package frc.robot.util.math;

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

    public static double getRealAngleDistanceSwerve(double current, double target, boolean cw) {
        switch (getQuadrantSwerve(current)) {
            case 1:
                if (cw) return target - current;
                else {
                    if (getQuadrantSwerve(target) == 1) return current - target;
                    else return (2 * PI - target) + current;
                }
            case 4:
                if (cw) return target - current;
                else {
                    if (getQuadrantSwerve(target) == 1 || getQuadrantSwerve(target) == 4) return current - target;
                    else return (2 * PI - target) + current;
                }
            case 3:
                if (!cw) return current - target;
                else {
                    if (getQuadrantSwerve(target) == 2 || getQuadrantSwerve(target) == 3) return target - current;
                    else return (2 * PI - current) + target;
                }
            case 2:
                if (!cw) return current - target;
                else {
                    if (getQuadrantSwerve(target) == 2) return target - current;
                    else return (2 * PI - current) + target;
                }
        }
        return 0;
    }

    public static double getRealAngleDistanceAuto(double current, double target, boolean cw) {
        switch (getQuadrantAuto(current)) {
            case 1:
                if (!cw) return target - current;
                else {
                    if (getQuadrantAuto(target) == 1) return current - target;
                    else return (2 * PI - target) + current;
                }
            case 4:
                if (cw) return current - target;
                else {
                    if (getQuadrantAuto(target) == 4) return target - current;
                    else return (2 * PI - current) + target;
                }
            case 3:
                if (cw) return current - target;
                else {
                    if (getQuadrantAuto(target) == 4 || getQuadrantAuto(target) == 3) return target - current;
                    else return (2 * PI - current) + target;
                }
            case 2:
                if (!cw) return target - current;
                else {
                    if (getQuadrantAuto(target) == 2 || getQuadrantAuto(target) == 1) return current - target;
                    else return (2 * PI - target) + current;
                }
        }
        return 0;
    }

    public static int getQuadrantSwerve(double angle) {
        angle = Angle.standardize(angle);
        if (angle >= 0 && angle < PI / 2) return 1;
        if (angle >= PI / 2 && angle < PI) return 4;
        if (angle >= PI && angle < 3 * PI / 2) return 3;
        if (angle >= 3 * PI / 2 && angle < 2 * PI) return 2;
        return -6;
    }

    public static int getQuadrantAuto(double angle) {
        angle = Angle.standardize(angle);
        if (angle >= 0 && angle < PI / 2) return 1;
        if (angle >= PI / 2 && angle < PI) return 2;
        if (angle >= PI && angle < 3 * PI / 2) return 3;
        if (angle >= 3 * PI / 2 && angle < 2 * PI) return 4;
        return -6;
    }

    @Override
    public String toString() {
        return "(" + radians + " radians" + ")";
    }
}
