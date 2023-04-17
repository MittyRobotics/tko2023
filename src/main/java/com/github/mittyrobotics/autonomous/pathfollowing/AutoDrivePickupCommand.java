package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.SwervePath;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDrivePickupCommand extends CommandBase {
    private SwervePath path;
    private double linearThreshold, angularThreshold, maxW;
    private Pose robot;
    private double speed = 0, targetAngle;

    private int index;
    private double dt, lastT;

    public AutoDrivePickupCommand(double linearThreshold, double angularThreshold, int index, SwervePath path, double maxW) {
        this.path = path;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        this.index = index;
        this.maxW = maxW;

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();

        path.changeSpline(ArmKinematics.getSplineToGamePiece());

        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        dt = Timer.getFPGATimestamp() - lastT;

        path.changeSpline(ArmKinematics.getSplineToGamePiece());

        Pose robot = Odometry.getInstance().getState();
        double heading = Gyro.getInstance().getHeadingRadians();

        Vector linear = path.updateLinear(robot, dt);

        double norm = SwerveSubsystem.standardize(heading);
        double normDes = SwerveSubsystem.standardize(
                path.spline.getVelocityVector(0).getAngle().getRadians());

        double angularVel = SwerveSubsystem.getDesiredAngularMP(
                norm, normDes, maxW, maxW, 0.02
        );

        SwerveSubsystem.getInstance().setSwerveInvKinematics(linear, angularVel);
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
        return false;
//        return StateMachine.getInstance().getIntakingState() == StateMachine.IntakeState.STOW;
    }
}
