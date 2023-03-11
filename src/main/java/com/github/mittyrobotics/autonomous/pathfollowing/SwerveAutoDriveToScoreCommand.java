package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.github.mittyrobotics.autonomous.pathfollowing.PathFollowingConstants.*;

public class SwerveAutoDriveToScoreCommand extends CommandBase {
    private SwervePath path;
    private double linearThreshold, angularThreshold;
    private Pose robot;
    private PIDController angularController;
    private double speed = 0, targetAngle;

    boolean zeroOnEnd;

    public SwerveAutoDriveToScoreCommand(double linearThreshold, double angularThreshold, boolean zeroOnEnd, SwervePath path) {
        setName("Swerve Pure Pursuit");
        this.path = path;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        this.angularController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

        this.zeroOnEnd = zeroOnEnd;

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
        speed = path.getInitSpeed();
        targetAngle = path.getEndHeading().getRadians();
        robot = Odometry.getInstance().getState();

        System.out.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nSTARTED");
    }

    @Override
    public void execute() {
        robot = Odometry.getInstance().getState();
        System.out.println("Target: " + path.getByT(1.0).getPosition());

        angularController = new PIDController(path.getKp(), path.getKi(), path.getKd());

        double leftY = OI.getInstance().getDriveController().getLeftX();
        double leftX = -OI.getInstance().getDriveController().getLeftY();
        speed = Math.sqrt(leftY * leftY + leftX * leftX) * SwerveConstants.MAX_LINEAR_VEL;

        double closest = path.getSpline().getClosestPoint(robot, 50, 10);

        double heading = robot.getHeading().getRadians();
        targetAngle = new Vector(robot.getPosition(), path.getByT(1.0).getPosition()).getAngle().getRadians();
        double angle = targetAngle - heading;
        System.out.println("SPEED: " + speed);
        Vector linearVel = new Vector(new Angle(angle), speed);

        double angularVel = angularController.calculate(robot.getHeading().getRadians(), path.getHeadingAtLookahead(robot, path.getLookahead()).getRadians());
        double currentAngle = robot.getHeading().getRadians();
        double desiredAngle = path.getHeadingAtLookahead(robot, path.getLookahead()).getRadians();

        if (Math.abs(angularVel) < path.getMinAngular()) {
            angularVel = (angularVel > 0) ? path.getMinAngular() : -path.getMinAngular();
            angularVel = ((desiredAngle - currentAngle) > angularThreshold * closest) ? angularVel : 0;
        }

        System.out.println(linearVel + " " + (-angularVel));
        if (Double.isNaN(angularVel)) angularVel = 0;
        SwerveSubsystem.getInstance().setSwerveInvKinematics(linearVel, angularVel);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
    }

    @Override
    public void end(boolean interrupted) {
        if (zeroOnEnd) SwerveSubsystem.getInstance().setZero();
        SmartDashboard.putBoolean("Halted", true);
//        System.out.println("FINISHED COMMAND\n\n\n\n\n");
    }

    @Override
    public boolean isFinished() {
        System.out.println(new Vector(robot.getPosition(), path.getByT(1.0).getPosition()).getMagnitude() + "-error");
//        System.out.println(path.getByT(1.0).getPosition());
        return new Vector(robot.getPosition(), path.getByT(1.0).getPosition()).getMagnitude() < linearThreshold ;
    }
}
