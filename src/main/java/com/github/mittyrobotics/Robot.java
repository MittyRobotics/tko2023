package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.routines.*;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.intake.IntakeSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * 6trt * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public String auto = "preload";
    public boolean balance = true;
    public int tag, index;


    @Override
    public void robotInit() {
        LedSubsystem.getInstance().initHardware();
        Gyro.getInstance().initHardware();
        SwerveSubsystem.getInstance().initHardware();

        TelescopeSubsystem.getInstance().initHardware();
        PivotSubsystem.getInstance().initHardware();
        IntakeSubsystem.getInstance().initHardware();

        PivotSubsystem.getInstance().setBrakeMode();
        TelescopeSubsystem.getInstance().setBrakeMode();

        Odometry.getInstance().FIELD_LEFT_SIDE = false;
        double x = 600;
        double y = 20;
        double t = 0;
        Gyro.getInstance().setAngleOffset(t);
        Odometry.getInstance().setState(x, y, t);
        SwerveSubsystem.getInstance().setPose(new Pose(new Point(0, 0),
                new Angle(Gyro.getInstance().getHeadingRadians())));
        Odometry.getInstance().setScoringCam(true);
    }

    @Override
    public void robotPeriodic() {
        SwerveSubsystem.getInstance().updateForwardKinematics();

        //UPDATE FROM BEAGLE AND JETSON
        Odometry.getInstance().update();
        ArmKinematics.updateAngleToGamePiece(StateMachine.getInstance().getCurrentPieceState()
                == StateMachine.PieceState.CONE, 0);

        LoggerInterface.getInstance().put("Heading", Gyro.getInstance().getHeadingRadians());
        LoggerInterface.getInstance().put("Pose", Odometry.getInstance().getState());

        LoggerInterface.getInstance().put("Pose X", Odometry.getInstance().getPose()[0]);
        LoggerInterface.getInstance().put("Pose Y", Odometry.getInstance().getPose()[1]);

//        LoggerInterface.getInstance().put("Spark Current", IntakeSubsystem.getInstance().getCurrent());
//        System.out.println("CURRENT: " + IntakeSubsystem.getInstance().getCurrent());

        LoggerInterface.getInstance().put("Auto Mode", auto + " | balance " + balance +
                " | tag " + tag + " | index" + index);

        CommandScheduler.getInstance().run();

    }

    @Override
    public void autonomousInit() {
//        Odometry.getInstance().FIELD_LEFT_SIDE = LoggerInterface.getInstance().getValue("fieldside").equals("left");

        /* REAL CODE
        auto = LoggerInterface.getInstance().getValue("auto");
        balance = LoggerInterface.getInstance().getValue("autobalance").equals("true");

        switch(auto) {
            case "preload":
                tag = Integer.parseInt(LoggerInterface.getInstance().getValue("autotag"));
                index = Integer.parseInt(LoggerInterface.getInstance().getValue("autoindex"));
                new PreloadAndBalanceAuto(Odometry.getInstance().FIELD_LEFT_SIDE, tag, index, balance).schedule();
                break;
            case "low":
                new LowPlusConeAuto(Odometry.getInstance().FIELD_LEFT_SIDE, balance).schedule();
                break;
            case "high":
                new HighPlusConeCubeAuto(Odometry.getInstance().FIELD_LEFT_SIDE, balance).schedule();
                break;
        }

         */

        Odometry.getInstance().FIELD_LEFT_SIDE = false;

        new PrePlusOneAuto(true, false, StateMachine.PieceState.CUBE, true).schedule();

//        Pose init = Odometry.getInstance().getState();
//        Pose end = new Pose(Point.add(init.getPosition(), new Point(150, 100)), init.getHeading());
//        SwervePath path = new SwervePath(new QuinticHermiteSpline(init, end),
//                15, 3, 3, 3, 0, 0, true);
//        new PathFollowingCommand(path, init.getHeading().getRadians() + Math.PI, 5, 0.05,
//                0.1, 0.8, 6, 0, 0.02).schedule();

//        new AutoScoreCommand(8, 0, 3, 3, 3, 0, 0).schedule();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
//        LoggerInterface.getInstance().put("AUTO PICKUP", false);

        SwerveSubsystem.getInstance().setRampRate(0.5);
        OI.getInstance().setupControls();
        Odometry.getInstance().disableCustomCam();
        Odometry.getInstance().setScoringCam(false);
        OI.getInstance().zeroAll();
        StateMachine.getInstance().setIntakeOff();


//        new InitAutoCommand(new Pose(new Point(40.45 + 32, 42.19 - 20.873), new Angle(Math.PI))).schedule();

//        SwerveSubsystem.getInstance().setRampRate(0);
//        new FastOvershootBalance(3, 0.7, false).schedule();
//        new TimedOvershootBalance(3, 1000, 0.7, false).schedule();
        //for false
//        new TimedBangBangBalance(2.5, 650, 0.3, false).schedule();
        //for true
//        new TimedBangBangBalance(2, 700, 0.5, true).schedule();


    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
//        LoggerInterface.getInstance().put("AGNEL TO GP", ArmKinematics.getSplineToGamePiece());
//    System.out.println("ANGLE: " + PivotSubsystem.getInstance().getPositionRadians());
//    System.out.println("RADIUS: "  + TelescopeSubsystem.getInstance().getDistanceMeters());
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        TelescopeSubsystem.getInstance().setBrakeMode();
        PivotSubsystem.getInstance().setBrakeMode();

        LedSubsystem.getInstance().turnOff();

//        SwerveSubsystem.getInstance().setAnglesZero();
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
//        System.out.println(LoggerInterface.getInstance().getValue("fieldside"));
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
