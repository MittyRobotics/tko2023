package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.Limelight;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathFollowingCommand extends CommandBase {

    protected SwervePath path;
    private PIDController controller;
    protected double lastTime, endHeading, linearThreshold, angularThreshold, startingHeading, angStart, angEnd;
    private boolean useInterp;
    private double maxW;

    public PathFollowingCommand(SwervePath path, double endHeading, double linearThreshold, double angularThreshold,
                                double angStart, double angEnd, double kP_OR_MAXW, double kI, double kD, boolean useInterp) {

        addRequirements(SwerveSubsystem.getInstance());

        this.path = path;
        this.endHeading = endHeading;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        this.angStart = angStart;
        this.angEnd = angEnd;
        this.useInterp = useInterp;
        this.maxW = kP_OR_MAXW;
        this.controller = new PIDController(kP_OR_MAXW, kI, kD);
    }

    @Override
    public void initialize() {
        System.out.println("INIT\n\n\n");
        lastTime = Timer.getFPGATimestamp();
        startingHeading = Gyro.getInstance().getHeadingRadians();
    }

    @Override
    public void execute() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        
        com.github.mittyrobotics.util.math.Pose p = Odometry.getInstance().getState();
        Pose robot = new Pose(new Point(p.getPoint().getX(), p.getPoint().getY()), new Angle(p.getAngle().getRadians()));
        double heading = Gyro.getInstance().getHeadingRadians();


        Vector linear = path.updateLinear(robot, dt);

        double norm = (heading);
        double normDes, angularVel;
        if(useInterp) {
            normDes = com.github.mittyrobotics.util.math.Angle.standardize(path.getHeadingGoal(startingHeading, endHeading, angStart, angEnd));
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
            angularVel = controller.calculate(dist * (right ? -1 : 1), 0);
        }
        else {
            normDes = com.github.mittyrobotics.util.math.Angle.standardize(endHeading);
            // TODO: 9/21/2023 FIX
            angularVel = 0;
//            angularVel = SwerveSubsystem.getDesiredAngularMP(
//                    norm, normDes, maxW, maxW, 0.02
//            );
        }

        SwerveSubsystem.getInstance().calculateInputs(
                new com.github.mittyrobotics.util.math.Vector(linear.getX(), linear.getY()), angularVel);
        SwerveSubsystem.getInstance().applyCalculatedInputs();

        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDED\n\n\n");
//        SwerveSubsystem.getInstance().setZero();
    }

    @Override
    public boolean isFinished() {
//        LoggerInterface.getInstance().put("DESIRED", path.getGoal());
        com.github.mittyrobotics.util.math.Point p = Odometry.getInstance().getState().getPosition();
        return new Vector(new Point(p.getX(), p.getY()), path.getGoal()).getMagnitude() <= linearThreshold;
    }
}
