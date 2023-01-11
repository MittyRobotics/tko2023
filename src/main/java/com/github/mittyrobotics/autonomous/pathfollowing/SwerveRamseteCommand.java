package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.github.mittyrobotics.autonomous.pathfollowing.PathFollowingConstants.*;

public class SwerveRamseteCommand extends CommandBase {
    private SwervePath[] paths;
    private int currentPath = 0;
    private double linearThreshold, angularThreshold;
    private Pose robot;
    private PIDController angularController;
    private double speed = 0;
    private double dt, lastT = 0;

    public SwerveRamseteCommand(double linearThreshold, double angularThreshold, SwervePath... paths) {
//        addRequirements(SwerveSubsystem.getInstance());
        setName("Swerve Ramsete");
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

//        System.out.println(currentPath);
        dt = Timer.getFPGATimestamp() - lastT;
        robot = SwerveSubsystem.getInstance().getPose();
//        SmartDashboard.putString("pose", robot.getPosition().toString());
        if (paths[currentPath].getSpline().getClosestPoint(robot, 50, 10) >= 0.9) currentPath++;
        if (currentPath == paths.length) currentPath--;

        // TODO: 9/4/2022 Fill in lookahead properly
//        System.out.println(currentPath);
        speed += paths[currentPath].getAccel() * dt;

        double closest = paths[currentPath].getSpline().getClosestPoint(robot, 50, 10);
//        System.out.println("closest:        " + paths[currentPath].getByT(closest).getPosition());
        double length = paths[currentPath].getSpline().getLength(closest, 1.0, 17);
        double vi = Math.sqrt(
                paths[currentPath].getEndSpeed() * paths[currentPath].getEndSpeed() + 2 * paths[currentPath].getDecel() * length
        );

        speed = Math.min(speed, vi);

        double angularVel = angularController.calculate(robot.getHeading().getRadians(), paths[currentPath].getHeadingAtLookahead(robot, LOOKAHEAD).getRadians());
        double currentAngle = robot.getHeading().getRadians();
        double desiredAngle = paths[currentPath].getHeadingAtLookahead(robot, LOOKAHEAD).getRadians();

        if(Math.abs(angularVel) < paths[currentPath].getMinAngular()) {
            angularVel = angularVel > 0 ? paths[currentPath].getMinAngular() : -paths[currentPath].getMinAngular();
            angularVel = desiredAngle - currentAngle > 0.01 ? angularVel : 0;
        }

        double tForLookahead = paths[currentPath].getTForLookahead(robot, LOOKAHEAD);

        double b = 0, Z = 0;
        double k = 2 * Z * Math.sqrt(
                (speed * speed * paths[currentPath].getCurvature(tForLookahead) * paths[currentPath].getCurvature(tForLookahead)) +
                b * speed * speed
        );

        Vector vectorToLookahead = paths[currentPath].getVectorToLookahead(robot, LOOKAHEAD);
        Vector desiredDirectionOfMotion = new Vector(
                paths[currentPath].getByT(tForLookahead).getPosition(),
                paths[currentPath].getByT(tForLookahead).getPosition()
        );
        desiredDirectionOfMotion = new Vector(desiredDirectionOfMotion.getY(), desiredDirectionOfMotion.getX());
        Angle desiredTheta = desiredDirectionOfMotion.getAngle();

        double theta = SwerveSubsystem.getInstance().getDirectionOfTravel().getRadians();
        double ex = Math.cos(theta) * (vectorToLookahead.getX()) + Math.sin(theta) * (vectorToLookahead.getY());
        double ey = -Math.sin(theta) * (vectorToLookahead.getX()) + Math.cos(theta) * (vectorToLookahead.getY());
        double etheta = desiredTheta.getRadians() - theta;

        double v = speed * Math.cos(etheta) + k * ex;
        double w = speed * paths[currentPath].getCurvature(tForLookahead) + k * etheta + b * speed * Math.sin(etheta) / etheta * ey;

        if (new Vector(robot.getPosition(), paths[paths.length - 1].getByT(1.0).getPosition()).getMagnitude() < linearThreshold &&
                Math.abs(paths[paths.length - 1].getByT(1.0).getHeading().getRadians() - robot.getHeading().getRadians()) < angularThreshold &&
                currentPath == paths.length - 1) {
//            linearVel = new Vector(0, 0);
            angularVel = 0;
        }

//        SwerveSubsystem.getInstance().setSwerveModule(linearVel, -angularVel);
        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());

        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
//        SwerveSubsystem.getInstance().setSwerveModule(new Vector(robot.getHeading(), 0), 0);
    }

    @Override
    public boolean isFinished() {
//        robot = SwerveSubsystem.getInstance().getPose();
//        return new Vector(robot.getPosition(), paths[paths.length - 1].getByT(1.0).getPosition()).getMagnitude() < linearThreshold &&
//                Math.abs(paths[paths.length - 1].getByT(1.0).getHeading().getRadians() - robot.getHeading().getRadians()) < angularThreshold;
        return false;
    }
}
