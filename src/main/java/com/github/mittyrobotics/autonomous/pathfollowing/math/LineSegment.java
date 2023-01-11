package com.github.mittyrobotics.autonomous.pathfollowing.math;

public class LineSegment {
    private Point start, end;
    private double length, slope, mag;

    public LineSegment(Point start, Point end) {
        this.start = start;
        this.end = end;

        setSlope(
                (end.getY() - start.getY()) / (end.getX() - start.getX())
        );

        setLength(Math.sqrt(
                (end.getY() - start.getY()) * (end.getY() - start.getY()) +
                (end.getX() - start.getX()) * (end.getX() - start.getX())
        ));
    }

    public LineSegment(Point start, double length, double slope) {
        this.start = start;
        this.length = length;
        this.slope = slope;

        mag = Math.sqrt(slope * slope + 1 * 1);

        // length/mag = x
        setEnd(new Point(
                length / mag + start.getX(),
                length / mag * slope + start.getY()
        ));
    }

    public Point getStart() {
        return start;
    }

    public void setStart(Point start) {
        this.start = start;
    }

    public Point getEnd() {
        return end;
    }

    public void setEnd(Point end) {
        this.end = end;
    }

    public double getLength() {
        return length;
    }

    public void setLength(double length) {
        this.length = length;
    }

    public double getSlope() {
        return slope;
    }

    public void setSlope(double slope) {
        this.slope = slope;
    }

    @Override
    public String toString() {
        return "LineSegment{" +
                "start=" + start +
                ", end=" + end +
                ", length=" + length +
                ", slope=" + slope +
                '}';
    }

    // TODO: 7/30/2022 FIX PARAMETRIC FUNCS 
    public double x(double t) {
        return t * length;
    }

    public double y(double t) {
        return slope * (x(t) - start.getX()) + start.getY();
    }
}
