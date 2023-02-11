package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveAutoPickupCommand extends CommandBase {
    private SwervePath path;
    private int currentPathNumber = 0;
    private double threshold;
    private Pose robot;
    private double speed = 0;
    private double dt, lastT = 0;

    public SwerveAutoPickupCommand(double threshold, SwervePath path) {
        setName("Swerve Pure Pursuit");
        this.path = path;
        this.threshold = threshold;

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        speed = path.getInitSpeed();
        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        dt = Timer.getFPGATimestamp() - lastT;
        robot = SwerveSubsystem.getInstance().getPose();

        speed += path.getAccel() * dt;
        speed = Math.min(speed, path.getMaxSpeed());

        double closest = path.getSpline().getClosestPoint(robot, 50, 10);
        double length = path.getSpline().getLength(closest, 1.0, 17);
        double vi = Math.sqrt(path.getEndSpeed() * path.getEndSpeed() + 2 * path.getDecel() * length);
        speed = Math.min(vi, speed);

        Point lookahead = path.getLookaheadPoint(robot, path.getLookahead());

        double purePursuitRadius = getRadiusFromPoints(robot, lookahead);


        SwerveSubsystem.getInstance().setDiffDriveKinematics(speed, purePursuitRadius);

        if (new Vector(robot.getPosition(), path.getByT(1.0).getPosition()).getMagnitude() > threshold) {
            SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().getDiffDriveVels());
            SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().getDiffDriveAngles());
        } else {
            SwerveSubsystem.getInstance().setZero();
        }

        lastT = Timer.getFPGATimestamp();
    }


    public static double getRadiusFromPoints(Pose robot, Point lookahead) {
        double radius;

        Angle angleOfRadius = new Angle(Math.PI/2 - (robot.getHeading().getRadians() - Math.PI/2));
        Line radius1 = new Line(angleOfRadius, robot.getPosition());

        Line lineThroughPoints = new Line(robot.getPosition(), lookahead);

        if(Math.abs(lineThroughPoints.getSlope() - Math.tan(robot.getHeading().getRadians())) < 2e-9) {
            radius = Double.POSITIVE_INFINITY;
        } else {
            Point midpoint = new Point((robot.getPosition().getX() + lookahead.getX()) / 2, (robot.getPosition().getY() + lookahead.getY()) / 2);
            Line radius2 = lineThroughPoints.getTangentAtPoint(midpoint);

            Point center = Line.getIntersection(radius1, radius2);
            radius = Point.getDistance(center, lookahead);
        }

        return radius * orientationOfPoseAndPoint(robot, lookahead);
    }

    public static int orientationOfPoseAndPoint(Pose pose, Point point3) {
        Point point1 = pose.getPosition();
        Point point2 = new Point(point1.getX() + Math.cos(pose.getHeading().getRadians()), point1.getY() + Math.sin(pose.getHeading().getRadians()));

        double test = (point2.getY() - point1.getY()) * (point3.getX() - point2.getX()) -
                (point2.getX() - point1.getX()) * (point3.getY() - point2.getY());

        if(Math.abs(test) < 2e-9) return 0;

        return (test > 0) ? 1 : -1;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
