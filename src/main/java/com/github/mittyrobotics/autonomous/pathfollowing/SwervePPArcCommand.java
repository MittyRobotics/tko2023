package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.github.mittyrobotics.autonomous.pathfollowing.PathFollowingConstants.*;

public class SwervePPArcCommand extends CommandBase {
    private SwervePath[] paths;
    private int currentPath = 0;
    private double linearThreshold, angularThreshold;
    private Pose robot;
    private PIDController angularController;
    private double speed = 0;
    private double dt, lastT = 0;
    private double radius, cX, cY, centerNumerator, centerDenominator, f = 0; //f = poseDerivative
    private Point center;
    private Circle arcCircle;

    public SwervePPArcCommand(double linearThreshold, double angularThreshold, SwervePath... paths) {
        setName("Swerve Pure Pursuit");
        this.paths = paths;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        this.angularController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
        SwerveSubsystem.getInstance().resetPose();
        speed = paths[currentPath].getInitSpeed();
        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        angularController = new PIDController(paths[currentPath].getKp(), paths[currentPath].getKi(), paths[currentPath].getKd());

        dt = Timer.getFPGATimestamp() - lastT;
        robot = SwerveSubsystem.getInstance().getPose();

        double a = robot.getPosition().getX();
        double b = robot.getPosition().getY();
        double c = paths[currentPath].getByT(paths[currentPath].getTForLookahead(robot, LOOKAHEAD)).getPosition().getX();
        double d = paths[currentPath].getByT(paths[currentPath].getTForLookahead(robot, LOOKAHEAD)).getPosition().getY();
        double f = SwerveSubsystem.getInstance().getDirectionOfTravel().getRadians();

        f = (f == 0) ? f += 0.00000000001 : f;
        c = ((a == b) && (c == d)) ? c += 0.00000000001 : c;

        centerNumerator = -f*b*b - 2*a*b + 2*f*d*b - f*c*c + f*a*a - f*d*d + 2*a*d;
        centerDenominator = 2*(d - b - f*c + f*a);
        cX = centerNumerator/centerDenominator;
        cY = -(1/f) * (cX - a) + b;
        radius = Math.sqrt(Math.pow((a - cX), 2) + Math.pow((b - cY), 2));
        center = new Point(cX, cY);
        arcCircle = new Circle(center, radius);
//        arcCircle.getVectorToLookAhead(robot, LOOKAHEAD);

        if (paths[currentPath].getSpline().getClosestPoint(robot, 50, 10) >= 0.9) currentPath++;
        if (currentPath == paths.length) currentPath--;

        speed += paths[currentPath].getAccel() * dt;

        double closest = paths[currentPath].getSpline().getClosestPoint(robot, 50, 10);
        double length = paths[currentPath].getSpline().getLength(closest, 1.0, 17);
        double vi = Math.sqrt(
                paths[currentPath].getEndSpeed() * paths[currentPath].getEndSpeed() + 2 * paths[currentPath].getDecel() * length
        );

        speed = Math.min(speed, vi);

        Vector vectorToLookahead = paths[currentPath].getVectorToLookahead(robot, LOOKAHEAD);
        Vector linearVel = new Vector(vectorToLookahead.getY(), vectorToLookahead.getX());

        double angularVel = angularController.calculate(robot.getHeading().getRadians(), paths[currentPath].getHeadingAtLookahead(robot, LOOKAHEAD).getRadians());
        double currentAngle = robot.getHeading().getRadians();
        double desiredAngle = paths[currentPath].getHeadingAtLookahead(robot, LOOKAHEAD).getRadians();

        if(Math.abs(angularVel) < paths[currentPath].getMinAngular()) {
            angularVel = (angularVel > 0) ? paths[currentPath].getMinAngular() : -paths[currentPath].getMinAngular();
            angularVel = ((desiredAngle - currentAngle) > 0.01) ? angularVel : 0;
        }

        double heading = Gyro.getInstance().getHeadingRadians();
        double angle = Math.atan2(linearVel.getY(), linearVel.getX()) + heading;

        linearVel = new Vector(new Angle(angle), speed);

        if (new Vector(robot.getPosition(), paths[paths.length - 1].getByT(1.0).getPosition()).getMagnitude() < linearThreshold &&
                Math.abs(paths[paths.length - 1].getByT(1.0).getHeading().getRadians() - robot.getHeading().getRadians()) < angularThreshold &&
                currentPath == paths.length - 1) {
            linearVel = new Vector(0, 0);
            angularVel = 0;
        }

        SwerveSubsystem.getInstance().setSwerveModule(linearVel, -angularVel);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());

        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());

        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setSwerveModule(new Vector(0, 0), 0);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());

        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
