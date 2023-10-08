package frc.robot.commands;

import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.util.autonomous.SwervePath;
import frc.robot.util.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathFollowingCommand extends CommandBase {
    private Gyro gyro;
    private Swerve swerve;
    private PoseEstimator poseEstimator;

    private SwervePath path;
    private Pose robot;
    private double dt, lastTime, curVel;
    private PIDController angularController;

    public PathFollowingCommand(Gyro gyro, Swerve swerve, PoseEstimator poseEstimator, SwervePath path) {
        this.gyro = gyro;
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;

        this.path = path;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        robot = poseEstimator.getState();
        lastTime = Timer.getFPGATimestamp();
        curVel = path.getInitSpeed();
        angularController = new PIDController(path.getKp(), path.getKi(), path.getKd());
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("ended", 0);
        dt = Timer.getFPGATimestamp() - lastTime;
        robot = poseEstimator.getState();

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

//        double desiredHeading = Angle.standardize(path.getCurrentDesiredHeading(currentLength).getRadians());
//        double currentHeading = Angle.standardize(Gyro.getInstance().getHeadingRadians());
//        boolean cw = (desiredHeading - currentHeading < PI && desiredHeading - currentHeading > 0)
//                || desiredHeading - currentHeading < -PI;
//        double angleDist = Angle.getRealAngleDistanceAuto(currentHeading, desiredHeading, cw);

//        double angularVel = angularController.calculate(currentHeading, currentHeading + (cw ? -1 : 1) * angleDist);
        double angularVel = angularController.calculate(gyro.getHeadingRadians(),
                path.getCurrentDesiredHeading(currentLength).getRadians());

        SmartDashboard.putNumber("desired angle", path.getCurrentDesiredHeading(currentLength).getDegrees());
        SmartDashboard.putNumber("dist to end", new Vector(robot.getPoint(), path.getByT(1.0).getPoint()).getMagnitude());


//        SwerveSubsystem.getInstance().calculateInputs(linearVel, 0);
//        SwerveSubsystem.getInstance().calculateInputs(new Vector(-100, 0), 0);
        swerve.calculateInputs(linearVel, angularVel);
        swerve.applyCalculatedInputs();

        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDED \n\n\n\n");
        SmartDashboard.putNumber("ended", 1);
        SmartDashboard.putString("end pos", path.getByT(1.0).toString());
        swerve.setZero();
    }

    @Override
    public boolean isFinished() {
        return new Vector(robot.getPoint(), path.getByT(1.0).getPoint()).getMagnitude() <= 5;
    }
}
