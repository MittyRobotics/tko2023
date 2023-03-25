package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.LoggerInterface;
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
    private double lastTime, endHeading, linearThreshold, angularThreshold, startingHeading, angStart, angEnd;
    private boolean useInterp;

    public PathFollowingCommand(SwervePath path, double endHeading, double linearThreshold, double angularThreshold,
                                double angStart, double angEnd, double kP, double kI, double kD, boolean useInterp) {

        addRequirements(SwerveSubsystem.getInstance());

        this.path = path;
        this.endHeading = endHeading;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        this.angStart = angStart;
        this.angEnd = angEnd;
        this.useInterp = useInterp;
        angularController = new PIDController(kP, kI, kD);
    }

    @Override
    public void initialize() {
        lastTime = Timer.getFPGATimestamp();
        startingHeading = Gyro.getInstance().getHeadingRadians();
    }

    @Override
    public void execute() {
        double dt = Timer.getFPGATimestamp() - lastTime;

        Pose robot = Odometry.getInstance().getState();
        double heading = Gyro.getInstance().getHeadingRadians();

//        System.out.println(dt + "   " + curVel);
        Vector linear = path.updateLinear(robot, dt);

//        System.out.println(linear);

        double norm = SwerveSubsystem.standardize(heading);
        double normDes;
        if(useInterp)
            normDes = SwerveSubsystem.standardize(path.getHeadingGoal(startingHeading, endHeading, angStart, angEnd));
        else normDes = SwerveSubsystem.standardize(endHeading);

        boolean right;
        double dist;

        if (normDes < norm) {
            if (norm - normDes > Math.PI) {
                right = true;
                dist = normDes + 2 * Math.PI - norm;
            } else {
                right = false;
                dist = norm - normDes;
            }
        } else {
            if (normDes - norm > Math.PI) {
                right = false;
                dist = norm + 2 * Math.PI - normDes;
            } else {
                right = true;
                dist = normDes - norm;
            }
        }

        double angularVel = -angularController.calculate(dist * (right ? 1 : -1), 0);

//        System.out.println("SEFKJSHEKFESF " + dist);
        SwerveSubsystem.getInstance().setSwerveInvKinematics(linear, angularVel);
        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());

        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
//        SwerveSubsystem.getInstance().setZero();
    }

    @Override
    public boolean isFinished() {
        LoggerInterface.getInstance().put("DESIRED", path.getGoal());
        return new Vector(Odometry.getInstance().getState().getPosition(), path.getGoal()).getMagnitude() <= linearThreshold
                && Math.abs(Gyro.getInstance().getHeadingRadians() - endHeading) <= angularThreshold;
    }
}
