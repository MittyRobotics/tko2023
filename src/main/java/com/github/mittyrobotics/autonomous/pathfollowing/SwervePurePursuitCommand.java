package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.github.mittyrobotics.autonomous.pathfollowing.PathFollowingConstants.*;

public class SwervePurePursuitCommand extends CommandBase {
    private SwervePath[] paths;
    private int currentPathNumber = 0;
    private SwervePath currentPath;
    private double linearThreshold, angularThreshold;
    private Pose robot;
    private PIDController angularController;
    private double speed = 0;
    private double dt, lastT = 0;

    public SwervePurePursuitCommand(double linearThreshold, double angularThreshold, SwervePath... paths) {
//        addRequirements(SwerveSubsystem.getInstance());
        setName("Swerve Pure Pursuit");
        this.paths = paths;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        this.angularController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
        SwerveSubsystem.getInstance().resetPose();
        speed = paths[currentPathNumber].getInitSpeed();
        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
//        System.out.println(currentPath);
        dt = Timer.getFPGATimestamp() - lastT;
        robot = SwerveSubsystem.getInstance().getPose();
//        SmartDashboard.putString("pose", robot.getPosition().toString());

        if (paths[currentPathNumber].getSpline().getClosestPoint(robot, 50, 10) >= 0.99) currentPathNumber++;
        if (currentPathNumber == paths.length) currentPathNumber--;

        currentPath = paths[currentPathNumber];
        angularController = new PIDController(currentPath.getKp(), currentPath.getKi(), currentPath.getKd());

        // TODO: 9/4/2022 Fill in lookahead properly
//        System.out.println(currentPath);
        speed += currentPath.getAccel() * dt;
        speed = Math.min(speed, currentPath.getMaxSpeed());

        double closest = currentPath.getSpline().getClosestPoint(robot, 50, 10);
        SmartDashboard.putNumber("Error", new Vector(robot.getPosition(), currentPath.getByT(closest).getPosition()).getMagnitude());
//        System.out.println("closest:        " + paths[currentPath].getByT(closest).getPosition());
        double length = currentPath.getSpline().getLength(closest, 1.0, 17);
        double vi = Math.sqrt(
                currentPath.getEndSpeed() * currentPath.getEndSpeed() + 2 * currentPath.getDecel() * length
        );

        SmartDashboard.putBoolean("Speed less than max", vi < speed);

        SmartDashboard.putString("closest point", currentPath.getByT(closest).getPosition().toString());
        Vector vectorToLookahead = currentPath.getVectorToLookahead(robot, currentPath.getLookahead());
        SmartDashboard.putString("vector to lookahead", vectorToLookahead.toString());

        //System.out.println(vectorToLookahead);
        // TODO: 9/4/2022 Fill in linear velocity scalar
//        double scalar = 0.5;
//        Vector linearVel = Vector.multiply(scalar, vectorToLookahead);
//        Vector linearVel = ((vectorToLookahead.getX() * scalar), );
        Vector linearVel = new Vector(vectorToLookahead.getY(), vectorToLookahead.getX());

        // TODO: 9/5/2022 Fill in angular PID constants (likely just P)
        double angularVel = angularController.calculate(robot.getHeading().getRadians(), currentPath.getHeadingAtLookahead(robot, currentPath.getLookahead()).getRadians());
        double currentAngle = robot.getHeading().getRadians();
        double desiredAngle = currentPath.getHeadingAtLookahead(robot, currentPath.getLookahead()).getRadians();
        SmartDashboard.putNumber("Desired Angle", desiredAngle);

        SmartDashboard.putNumber("Angular Velocity", angularVel);

//        double min = (angularVel > 0 && desiredAngle > currentAngle + 0.01)  ?  0.2 : (angularVel < 0 && desiredAngle + 0.01 < currentAngle) ? -0.2 : 0;
//        angularVel = Math.max(angularVel, min);
        if(Math.abs(angularVel) < currentPath.getMinAngular()) {
            angularVel = (angularVel > 0) ? currentPath.getMinAngular() : -currentPath.getMinAngular();
            angularVel = ((desiredAngle - currentAngle) > 0.01) ? angularVel : 0;
        }


//        double angularVel = (desiredAngle > currentAngle + 0.01) ? 0.6 : (desiredAngle + 0.01 < currentAngle) ? -0.6 : 0;

        SmartDashboard.putNumber("angular Vel", angularVel);
        SmartDashboard.putNumber("current Heading", robot.getHeading().getRadians());
        SmartDashboard.putNumber("desired Heading", currentPath.getHeadingAtLookahead(robot, currentPath.getLookahead()).getRadians());

//        System.out.println(linearVel);



//        double magnitude = linearVel.getMagnitude();

        double heading = Gyro.getInstance().getHeadingRadians();
//        double angle = Math.atan2(linearVel.getY(), linearVel.getX());
        double angle = Math.atan2(linearVel.getY(), linearVel.getX()) + heading;
//
//        System.out.println("lookAheadAngle     " + angle);
//
//        System.out.println("VTLA:               "  + vectorToLookahead);
//
//        System.out.println("linearVel:          " + linearVel);


        linearVel = new Vector(new Angle(angle), speed);
        System.out.println("SPEED: " + speed);
//        linearVel = new Vector(linearVel.getY(), linearVel.getX());
//        System.out.println(linearVel + " " + Math.atan2(linearVel.getY(), linearVel.getX()));

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putString("linear vel", linearVel.toString());
        SmartDashboard.putBoolean("Halted", false);
        SmartDashboard.putNumber("Distance to end", new Vector(robot.getPosition(), currentPath.getByT(1.0).getPosition()).getMagnitude());
        SmartDashboard.putNumber("Angle to end", Math.abs(currentPath.getByT(1.0).getHeading().getRadians() - robot.getHeading().getRadians()));

        if (new Vector(robot.getPosition(), paths[paths.length - 1].getByT(1.0).getPosition()).getMagnitude() < linearThreshold &&
                Math.abs(paths[paths.length - 1].getByT(1.0).getHeading().getRadians() - robot.getHeading().getRadians()) < angularThreshold &&
                currentPathNumber == paths.length - 1) {
            linearVel = new Vector(0, 0);
            angularVel = 0;
            SmartDashboard.putBoolean("Halted", true);
//            System.out.println("Halted: " + true);
//            System.out.println("Halted: " + true);
//            System.out.println("Halted: " + true);
//            System.out.println("Halted: " + true);
//            System.out.println("Halted: " + true);
//            System.out.println("Halted: " + true);

        }
//
//        System.out.println("VTLA1:               " + vectorToLookahead);
//
//        System.out.println("linearVel1:          " + linearVel);

        SwerveSubsystem.getInstance().setSwerveModule(linearVel, -angularVel);
//        SwerveSubsystem.getInstance().setSwerveModule(linearVel, 0);
//        SwerveSubsystem.getInstance().setSwerveModule(new Vector(0, 0), angularVel);


        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());

//        double[] angles = SwerveSubsystem.getInstance().desiredAngles();
//        for(int i = 0; i < angles.length; ++i) angles[i] -= Math.PI/2;
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
//        System.out.printf("currentpath: %d, closest: %f, lookahead: %s, robot: %s, linear: %s, angular: %f\n",
//                currentPath,
//                paths[currentPath].getSpline().getClosestPoint(robot, 50, 10),
//                vectorToLookahead,
//                robot,
//                linearVel,
//                angularVel);
//        System.out.println("Position: " + SwerveSubsystem.getInstance().getPose().toString());

        lastT = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
//        SwerveSubsystem.getInstance().setSwerveModule(new Vector(robot.getHeading(), 0), 0);
    }

    @Override
    public boolean isFinished() {
//        robot = SwerveSubsystem.getInstance().getPose();
//        return new Vector(robot.getPosition(), paths[paths.length - 1].getByT(1.0).getPosition()).getMagnitude() < linearThreshold &&
//                Math.abs(paths[paths.length - 1].getByT(1.0).getHeading().getRadians() - robot.getHeading().getRadians()) < angularThreshold;
        return false;
    }
}
