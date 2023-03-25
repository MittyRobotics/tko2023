package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathFollowingCommand extends CommandBase {

    private SwervePath path;
    private PIDController angularController;
    private double lastTime, endHeading, linearThreshold, angularThreshold;

    public PathFollowingCommand(SwervePath path, double endHeading, double linearThreshold, double angularThreshold,
                                double kP, double kI, double kD) {
        this.path = path;
        this.endHeading = endHeading;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        angularController = new PIDController(kP, kI, kD);
    }

    @Override
    public void initialize() {
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double dt = Timer.getFPGATimestamp() - lastTime;

        Pose robot = Odometry.getInstance().getState();
        double heading = Gyro.getInstance().getHeadingRadians();
        double curVel = SwerveSubsystem.getInstance().getDesiredVel().getMagnitude();

        Vector linear = path.updateLinear(robot, dt, curVel);

        double angular = -angularController.calculate(heading, endHeading);

        SwerveSubsystem.getInstance().setSwerveInvKinematics(linear, angular);
        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());

        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return new Vector(Odometry.getInstance().getState().getPosition(), path.getGoal()).getMagnitude() <= linearThreshold
                && Math.abs(Gyro.getInstance().getHeadingRadians() - endHeading) <= angularThreshold;
    }
}
