package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.github.mittyrobotics.autonomous.pathfollowing.PathFollowingConstants.*;

public class SwerveAutoPickupCommandv2 extends CommandBase {
    private SwervePath path;
    private double linearThreshold, angularThreshold;
    private Pose robot;
    private PIDController angularController;
    private double speed = 0;
    private double dt, lastT = 0;

    public SwerveAutoPickupCommandv2(double linearThreshold, double angularThreshold, SwervePath path) {
        setName("Swerve Pure Pursuit");
        this.path = path;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        this.angularController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
        speed = path.getInitSpeed();
        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        dt = Timer.getFPGATimestamp() - lastT;
        robot = SwerveSubsystem.getInstance().getPose();

        angularController = new PIDController(path.getKp(), path.getKi(), path.getKd());

        double leftY = OI.getInstance().getPS4Controller().getLeftX();
        double leftX = -OI.getInstance().getPS4Controller().getLeftY();
        speed += path.getAccel() * dt;
        speed = Math.min(speed, Math.sqrt(leftY * leftY + leftX * leftX) * SwerveConstants.MAX_LINEAR_VEL);

        double closest = path.getSpline().getClosestPoint(robot, 50, 10);
        double length = path.getSpline().getLength(closest, 1.0, 17);

        double vi = Math.sqrt(
                path.getEndSpeed() * path.getEndSpeed() + 2 * path.getDecel() * length
        );

        speed = Math.min(vi, speed);

        Vector vectorToEnd = new Vector(robot.getPosition(), path.getByT(1.0).getPosition());

        Vector linearVel = new Vector(vectorToEnd.getY(), vectorToEnd.getX());

        SmartDashboard.putNumber("closest", closest);

        double heading = Gyro.getInstance().getHeadingRadians();
        double angle = Math.atan2(linearVel.getY(), linearVel.getX()) + heading;
        double totalLength = path.getSpline().getLength(0, 1.0, 17);
        double offset = (robot.getHeading().getRadians() - angle > 0 ? 1 : -1) * Math.PI/3 * (totalLength - length) / totalLength;
        linearVel = new Vector(new Angle(angle + offset), speed);

        double angularVel = angularController.calculate(robot.getHeading().getRadians(), path.getHeadingAtLookahead(robot, path.getLookahead()).getRadians());
        double currentAngle = robot.getHeading().getRadians();
        double desiredAngle = path.getHeadingAtLookahead(robot, path.getLookahead()).getRadians();

        if (Math.abs(angularVel) < path.getMinAngular()) {
            angularVel = (angularVel > 0) ? path.getMinAngular() : -path.getMinAngular();
            angularVel = ((desiredAngle - currentAngle) > angularThreshold/closest) ? angularVel : 0;
        }

        //Can probably be removed
        if (new Vector(robot.getPosition(), path.getByT(1.0).getPosition()).getMagnitude() < linearThreshold &&
                Math.abs(path.getByT(1.0).getHeading().getRadians() - robot.getHeading().getRadians()) < angularThreshold) {
            linearVel = new Vector(0, 0);
            angularVel = 0;
            SmartDashboard.putBoolean("Halted", true);
        } else {
            SmartDashboard.putBoolean("Halted", false);
        }

        SwerveSubsystem.getInstance().setSwerveInvKinematics(linearVel, -angularVel);
//        SwerveSubsystem.getInstance().setSwerveInvKinematics(linearVel, 0);
//        SwerveSubsystem.getInstance().setSwerveInvKinematics(new Vector(0, 0), angularVel);

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
        return new Vector(robot.getPosition(), path.getByT(1.0).getPosition()).getMagnitude() < linearThreshold &&
                Math.abs(path.getByT(1.0).getHeading().getRadians() - robot.getHeading().getRadians()) < angularThreshold;
    }
}
