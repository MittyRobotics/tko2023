package com.github.mittyrobotics.autonomous.pathfollowing.math;

import com.github.mittyrobotics.drivetrain.SwerveSubsystem;

public class Circle {
    private Point center;
    private double radius;

    public Circle(Point center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    public Point getCenter() {
        return center;
    }

    public void setCenter(Point center) {
        this.center = center;
    }

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public double circumference(double angle) {
        return angle * getRadius();
    }

    public double area(double angle) {
        return angle / 2. * getRadius() * getRadius();
    }

    public Vector getVectorToLookahead(Point robot, Angle angle, double lookahead) {
        double theta = com.github.mittyrobotics.util.math.Angle.standardize(Math.atan2(robot.getY(), robot.getX()));

        int quadrant = 1;
        if (robot.getX() >= 0) {
            if (robot.getY() >= 0) quadrant = 1;
            else quadrant = 4;
        } else {
            if (robot.getY() >= 0) quadrant = 2;
            else quadrant = 3;
        }

        boolean positive = true;
        switch (quadrant) {
            case 1:
                positive = angle.getRadians() >= Math.PI/2 || angle.getRadians() <= Math.PI;
            case 2:
                positive = angle.getRadians() >= Math.PI || angle.getRadians() <= 3*Math.PI/2;
            case 3:
                positive = angle.getRadians() >= 3*Math.PI/2 || angle.getRadians() <= 2*Math.PI;
            case 4:
                positive = angle.getRadians() >= 0 || angle.getRadians() <= Math.PI/2;
        }

        double phi = lookahead / radius;

        Point lookaheadPoint = positive ?
                new Point(radius * Math.cos(theta + phi), radius * Math.sin(theta + phi)) :
                new Point(radius * Math.cos(theta - phi), radius * Math.sin(theta - phi));
        return new Vector(robot, lookaheadPoint);
    }
}
