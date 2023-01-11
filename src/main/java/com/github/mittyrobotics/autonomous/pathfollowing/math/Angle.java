package com.github.mittyrobotics.autonomous.pathfollowing.math;

public class Angle {
    private double angle;

    public Angle(double angle) {
        this.angle = angle;
    }

    public double getRadians() {
        return angle;
    }

    public void setRadians(double angle) {
        this.angle = angle;
    }

    @Override
    public String toString() {
        return angle + " radians";
    }
}
