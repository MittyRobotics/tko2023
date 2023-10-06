package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.math.Pose;
import com.github.mittyrobotics.util.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putNumber("ended", 0);
        dt = Timer.getFPGATimestamp() - lastTime;
        robot = Odometry.getInstance().getState();

        //get total length traversed
        double closestT = path.getSpline().getClosestPoint(robot, 50, 7);
        SmartDashboard.putNumber("closestT", closestT);
        SmartDashboard.putString("closest pt", path.getByT(closestT).getPoint().toString());
        double currentLength = path.getSpline().getLength(0, closestT, 17);

        //get linear velocity direction - linear combination of tangent and error vectors
        Vector errorVector = path.getErrorVector(robot);
        Vector tangentVector = path.getSpline().getVelocityVector(closestT);
        Vector linearDirection = Vector.add(
                Vector.multiply(path.getCorrectionScale(), errorVector),
                Vector.multiply(path.getTangentScale(), tangentVector)
        );
        linearDirection = Vector.multiply(1. / linearDirection.getMagnitude(), linearDirection);

        SmartDashboard.putString("errorVector", errorVector.toString());
        SmartDashboard.putString("tangentVector", tangentVector.toString());

        //accelerate to max velocity
        curVel += path.getAccel() * dt;
        curVel = Math.min(curVel, path.getMaxSpeed());

        //decelerate when nearing end of path
        double vi = Math.sqrt(path.getEndSpeed() * path.getEndSpeed() +
                2 * path.getDecel() * (path.getSpline().getLength() - currentLength));
        curVel = Math.min(curVel, vi);

        SmartDashboard.putNumber("vel", curVel);

        //scale linear velocity to motion profile
        Vector linearVel = Vector.multiply(curVel, linearDirection);

        SmartDashboard.putString("linearVel", linearVel.toString());

        double desiredHeading = Angle.standardize(path.getCurrentDesiredHeading(currentLength).getRadians());
        double currentHeading = Angle.standardize(Gyro.getInstance().getHeadingRadians());
        boolean cw = (desiredHeading - currentHeading < PI && desiredHeading - currentHeading > 0)
                || desiredHeading - currentHeading < -PI;
        double angleDist = Angle.getRealAngleDistanceAuto(currentHeading, desiredHeading, cw);

//        double angularVel = angularController.calculate(currentHeading, currentHeading + (cw ? -1 : 1) * angleDist);
        double angularVel = angularController.calculate(Gyro.getInstance().getHeadingRadians(),
                path.getCurrentDesiredHeading(currentLength).getRadians());


//        SwerveSubsystem.getInstance().calculateInputs(linearVel, 0);
//        SwerveSubsystem.getInstance().calculateInputs(new Vector(-100, 0), 0);
        SwerveSubsystem.getInstance().calculateInputs(linearVel, angularVel);
        SwerveSubsystem.getInstance().applyCalculatedInputs();

        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDED \n\n\n\n");
        SmartDashboard.putNumber("ended", 1);
        SmartDashboard.putString("end pos", path.getByT(1.0).toString());
        SwerveSubsystem.getInstance().setZero();
    }

    @Override
    public boolean isFinished() {
        return new Vector(robot.getPoint(), path.getByT(1.0).getPoint()).getMagnitude() <= 5;
    }
}
