package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
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

public class AutoDriveScoreCommand extends CommandBase {
    private SwervePath path;
    private double linearThreshold, angularThreshold;
    private Pose robot;
    private PIDController angularController;
    private double speed = 0, targetAngle;

    private boolean firstPath, auto;
    private double dt, lastT;

    public AutoDriveScoreCommand(double linearThreshold, double angularThreshold, boolean firstPath, SwervePath path, boolean auto) {
        setName("Swerve Pure Pursuit");
        this.path = path;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        this.angularController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

        this.firstPath = firstPath;
        this.auto = auto;

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
        angularController = new PIDController(path.getKp(), path.getKi(), path.getKd());

        speed = path.getInitSpeed();
        targetAngle = path.getEndHeading().getRadians();
        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        dt = Timer.getFPGATimestamp() - lastT;

        robot = Odometry.getInstance().getState();

        if (!auto) {
            double leftY = OI.getInstance().getDriveController().getLeftX();
            double leftX = -OI.getInstance().getDriveController().getLeftY();
            speed = Math.sqrt(leftY * leftY + leftX * leftX) * SwerveConstants.MAX_LINEAR_VEL;
        } else {
            double curSpeed = SwerveSubsystem.getInstance().getDesiredVel().getMagnitude();
            double distanceToEnd = new Vector(robot.getPosition(), path.getByT(1.0).getPosition()).getMagnitude();

            speed = Math.min(
                    maxVelocityFromDistance(distanceToEnd),
                    Math.min(curSpeed + dt * path.getAccel(), path.getMaxSpeed())
            );
        }


        double heading = Gyro.getInstance().getHeadingRadians();
        //difference vector between goal point and robot, in world coordinates
        Point diff = Point.add(path.getByT(1.0).getPosition(),
                Point.multiply(-1, robot.getPosition()));
        //world angle of vector
        double diffA = Math.atan2(diff.getY(), diff.getX());
        //angle of vector relative to robot - desired minus current
        double goalA = diffA - heading;

//        System.out.println(goalA);

        Vector linearVel = new Vector(new Angle(goalA), speed);

        double norm = SwerveSubsystem.standardize(heading);
        double normDes = SwerveSubsystem.standardize(path.getByT(1.0).getHeading().getRadians());

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


//        SwerveSubsystem.getInstance().setSwerveInvKinematics(linearVel, 0);
        SwerveSubsystem.getInstance().setSwerveInvKinematics(linearVel, angularVel);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());

        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        if (!firstPath) SwerveSubsystem.getInstance().setZero();
        SmartDashboard.putBoolean("Halted", true);
    }

    @Override
    public boolean isFinished() {
        if (firstPath) return Math.abs(Odometry.getInstance().getState().getPosition().getY() - path.getByT(1.0).getPosition().getY()) < linearThreshold;
        return new Vector(Odometry.getInstance().getState().getPosition(), path.getByT(1.0).getPosition()).getMagnitude() < linearThreshold ;
    }

    public double maxVelocityFromDistance(double dist) {
        double dist_m = dist / 39.3701;
        return Math.sqrt(path.getEndSpeed() * path.getEndSpeed() + 2 * path.getDecel() * dist_m);
    }

}
