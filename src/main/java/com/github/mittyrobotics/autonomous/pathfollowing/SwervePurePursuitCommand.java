package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.github.mittyrobotics.autonomous.pathfollowing.PathFollowingConstants.*;

public class SwervePurePursuitCommand extends CommandBase {
    private OldSwervePath[] paths;
    private int currentPathNumber = 0;
    private OldSwervePath currentPath;
    private double linearThreshold, angularThreshold;
    private Pose robot;
    private PIDController angularController;
    private double speed = 0;
    private double dt, lastT = 0;

    public SwervePurePursuitCommand(double linearThreshold, double angularThreshold, OldSwervePath... paths) {
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
        speed = paths[currentPathNumber].getInitSpeed();
        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        dt = Timer.getFPGATimestamp() - lastT;

        robot = Odometry.getInstance().getState();

        double closest = paths[currentPathNumber].getSpline().getClosestPoint(robot, 50, 10);
        if (closest >= 0.98) {
            if (currentPathNumber < paths.length - 1) {
                currentPathNumber++;
                closest = paths[currentPathNumber].getSpline().getClosestPoint(robot, 50, 10);
            }
            SmartDashboard.putString("end pos" + currentPathNumber, SwerveSubsystem.getInstance().getPose().getPosition().toString());
        }

        currentPath = paths[currentPathNumber];
        angularController = new PIDController(currentPath.getKp(), currentPath.getKi(), currentPath.getKd());

        speed = Math.min(speed + currentPath.getAccel() * dt, currentPath.getMaxSpeed());
        double length = currentPath.getSpline().getLength(closest, 1.0, 17);

        double vi = Math.sqrt(
                currentPath.getEndSpeed() * currentPath.getEndSpeed() + 2 * currentPath.getDecel() * length / 39.3701
        );
        speed = Math.min(vi, speed);

        Vector vectorToLookahead = currentPath.getVectorToLookahead(robot, length, currentPath.getLookahead());
//        if (currentPath.getLookahead() < length) vectorToLookahead = currentPath.getVectorToLookahead(robot, length, currentPath.getLookahead());
//        else vectorToLookahead = new Vector(
//                robot.getPosition(),
//                Point.add(currentPath.getByT(1.0).getPosition(),
//                        Point.multiply((currentPath.getLookahead() - length) / currentPath.getSpline().getVelocityVector(1.0).getMagnitude(),
//                                new Point(currentPath.getSpline().getVelocityVector(1.0))))
//        );

        Vector linearVel = new Vector(vectorToLookahead.getX(), vectorToLookahead.getY());

        SmartDashboard.putNumber("current path", currentPathNumber);
        SmartDashboard.putNumber("closest", closest);

        double heading = Gyro.getInstance().getHeadingRadians();
        double angle = Math.atan2(linearVel.getY(), linearVel.getX()) - heading;
        linearVel = new Vector(new Angle(angle), speed);

        double currentAngle = Gyro.getInstance().getHeadingRadians();
        double desiredAngle = currentPath.getCurrentDesiredHeading(length).getRadians();
        double angularVel = angularController.calculate(currentAngle, desiredAngle);

        if(Math.abs(angularVel) < currentPath.getMinAngular()) {
            angularVel = (angularVel > 0) ? currentPath.getMinAngular() : -currentPath.getMinAngular();
            angularVel = ((desiredAngle - currentAngle) > angularThreshold * closest) ? angularVel : 0;
        }

        SwerveSubsystem.getInstance().setSwerveInvKinematics(linearVel, 0);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());

        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setZero();
        SmartDashboard.putBoolean("Halted", true);
    }

    @Override
    public boolean isFinished() {
        return new Vector(robot.getPosition(), paths[paths.length - 1].getByT(1.0).getPosition()).getMagnitude() < linearThreshold &&
                Math.abs(paths[paths.length - 1].getByT(1.0).getHeading().getRadians() - robot.getHeading().getRadians()) < angularThreshold &&
                currentPathNumber == paths.length - 1;
    }
}
