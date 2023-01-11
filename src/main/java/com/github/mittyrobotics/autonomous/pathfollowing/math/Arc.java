package com.github.mittyrobotics.autonomous.pathfollowing.math;

public class Arc {
    private Circle circle;
    private Point start, end;

    public Arc(Circle circle, Point start, Point end) {
        this.circle = circle;
        this.start = start;
        this.end = end;
    }

    public Circle getCircle() {
        return circle;
    }

    public Point getStart() {
        return start;
    }

    public Point getEnd() {
        return end;
    }
}
