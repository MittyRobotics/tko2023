package frc.robot.util.math;

public class Pose {
    private Point point;
    private Angle angle;

    public Pose(Point p, Angle a) {
        point = p;
        angle = a;
    }

    public Pose(double x, double y, double theta, boolean radians) {
        point = new Point(x, y);
        angle = new Angle(theta, radians);
    }

    public Point getPoint() {
        return point;
    }

    public Angle getAngle() {
        return angle;
    }

    public Point getPosition() {
        return point;
    }

    public Angle getHeading() {
        return angle;
    }

    @Override
    public String toString() {
        return "{" + point.toString() + ", " + angle.toString() + "}";
    }
}
