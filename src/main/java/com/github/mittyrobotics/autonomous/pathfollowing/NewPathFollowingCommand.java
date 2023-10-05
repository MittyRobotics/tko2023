package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.math.Pose;
import com.github.mittyrobotics.util.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static java.lang.Math.PI;

public class NewPathFollowingCommand extends CommandBase {
    SwervePath path;
    Pose robot;
    double dt, lastTime, curVel;
    PIDController angularController;

    public NewPathFollowingCommand(SwervePath path) {
        this.path = path;
    }

    @Override
    public void initialize() {
        path = path == null ? AutoPathManager.getCurrentPath() : path;
        robot = Odometry.getInstance().getState();
        lastTime = Timer.getFPGATimestamp();
        curVel = path.getInitSpeed();
        angularController = new PIDController(path.getKp(), path.getKi(), path.getKd());
    }

    @Override
    public void execute() {
        dt = Timer.getFPGATimestamp() - lastTime;
        robot = Odometry.getInstance().getState();

        //get total length traversed
        double closestT = path.getSpline().getClosestPoint(robot, 50, 10);
        double currentLength = path.getSpline().getLength(0, closestT, 17);

        //get linear velocity direction - linear combination of tangent and error vectors
        Vector errorVector = path.getErrorVector(robot);
        Vector tangentVector = path.getSpline().getVelocityVector(closestT);
        Vector linearDirection = Vector.add(
                Vector.multiply(path.getCorrectionScale(), errorVector),
                Vector.multiply(path.getTangentScale(), tangentVector)
        );
        linearDirection = Vector.multiply(1. / linearDirection.getMagnitude(), linearDirection);

        //accelerate to max velocity
        curVel += path.getAccel() * dt;
        curVel = Math.min(curVel, path.getMaxSpeed());

        //decelerate when nearing end of path
        double vi = Math.sqrt(path.getEndSpeed() * path.getEndSpeed() +
                2 * path.getDecel() * (path.getSpline().getLength() - currentLength));
        curVel = Math.min(curVel, vi);

        //scale linear velocity to motion profile
        Vector linearVel = Vector.multiply(curVel, linearDirection);

        double desiredHeading = Angle.standardize(path.getCurrentDesiredHeading(currentLength).getRadians());
        double currentHeading = Angle.standardize(Gyro.getInstance().getHeadingRadians());
        boolean cw = (desiredHeading - currentHeading < PI && desiredHeading - currentHeading > 0)
                || desiredHeading - currentHeading < -PI;
        double angleDist = Angle.getRealAngleDistance(currentHeading, desiredHeading, cw);

        double angularVel = angularController.calculate(currentHeading, currentHeading + (cw ? -1 : 1) * angleDist);


        SwerveSubsystem.getInstance().calculateInputs(linearVel, 0);
//        SwerveSubsystem.getInstance().calculateInputs(linearVel, angularVel);
        SwerveSubsystem.getInstance().applyCalculatedInputs();

        lastTime = Timer.getFPGATimestamp();
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
