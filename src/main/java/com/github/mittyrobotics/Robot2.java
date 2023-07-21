package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Limelight;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.PathFollowingCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.SwervePath;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * 6trt * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot2 extends TimedRobot {
    public String auto = "balance";
    public boolean low = false, bal = false;

    @Override
    public void robotInit() {
        PivotSubsystem.getInstance().setBrakeMode();
        TelescopeSubsystem.getInstance().setBrakeMode();

        Odometry.getInstance().FIELD_LEFT_SIDE = true;
        double x = 0;
        double y = 0;
        double t = 0;
        Gyro.getInstance().setAngleOffset(t);
        Odometry.getInstance().setState(x, y, t);
        SwerveSubsystem.getInstance().setPose(new Pose(new Point(0, 0),
                new Angle(Gyro.getInstance().getHeadingRadians())));
        Odometry.getInstance().setScoringCam(true);
        Limelight.init(new Pose(new Point(0, 0), new Angle(0)), 0);
    }

    @Override
    public void robotPeriodic() {
        Limelight.update();

        SwerveSubsystem.getInstance().updateForwardKinematics();

        //UPDATE FROM BEAGLE AND JETSON
        Odometry.getInstance().update();
        System.out.println(Odometry.getInstance().getState());

        ArmKinematics.updateAngleToGamePiece(StateMachine.getInstance().getCurrentPieceState()
                == StateMachine.PieceState.CONE, 0);

//        LoggerInterface.getInstance().put("Heading", Gyro.getInstance().getHeadingRadians());
//        LoggerInterface.getInstance().put("Pose", Odometry.getInstance().getState());
//
//        LoggerInterface.getInstance().put("Pose X", Odometry.getInstance().getPose()[0]);
//        LoggerInterface.getInstance().put("Pose Y", Odometry.getInstance().getPose()[1]);

//        LoggerInterface.getInstance().put("Spark Current", IntakeSubsystem.getInstance().getCurrent());
//        System.out.println("CURRENT: " + IntakeSubsystem.getInstance().getCurrent());

        CommandScheduler.getInstance().run();

    }

    @Override
    public void autonomousInit() {
//        Odometry.getInstance().FIELD_LEFT_SIDE = LoggerInterface.getInstance().getValue("fieldside").equals("left");

//        auto = LoggerInterface.getInstance().getValue("auto");
//        low = LoggerInterface.getInstance().getValue("autoside").equals("L");
//        bal = LoggerInterface.getInstance().getValue("autobal").equals("T");

//        switch(auto) {
//            case "preload":
//                new PreloadAndBalanceAuto(Odometry.getInstance().FIELD_LEFT_SIDE).schedule();
//                break;
//            case "balance":
//                new PPOneAuto(low, Odometry.getInstance().FIELD_LEFT_SIDE, StateMachine.PieceState.CUBE, bal).schedule();
//                break;
//            case "pick":
//                new PPTwoAuto(low, Odometry.getInstance().FIELD_LEFT_SIDE).schedule();
//                break;
//        }
//

//        Odometry.getInstance().FIELD_LEFT_SIDE = false;
//
//        new PPTwoAuto(false, false).schedule();

//        Pose init = Odometry.getInstance().getState();
//        Pose end = new Pose(Point.add(init.getPosition(), new Point(150, 100)), init.getHeading());
//        SwervePath path = new SwervePath(new QuinticHermiteSpline(init, end),
//                15, 3, 3, 3, 0, 0, true);
//        new PathFollowingCommand(path, init.getHeading().getRadians() + Math.PI, 5, 0.05,
//                0.1, 0.8, 6, 0, 0.02).schedule();
        boolean leftSide = false;

        Pose first = new Pose(new Point(576, 109), new Angle(0));
        Pose second = new Pose(Point.add(first.getPosition(), new Point(-20, 0)), first.getHeading());
        Pose third = new Pose(Point.add(second.getPosition(), new Point(-40, 20)), new Angle(0));

        double scoreHeading = 0;

        new SequentialCommandGroup(
                new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(
                                        new Pose(first.getPosition(), new Angle(leftSide ? Math.PI : 0)),
                                        new Pose(second.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                                10, 2, 5, 2, 0, 1, true
                        ), scoreHeading, 6, 1,
                        0.1, 0.6, 3, 0, 0.02, true
                ),
                new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(
                                        new Pose(second.getPosition(), new Angle(leftSide ? Math.PI : 0)),
                                        new Pose(third.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                                10, 2, 5, 2, 0, 1, true
                        ), scoreHeading, 6, 1,
                        0.1, 0.6, 3, 0, 0.02, true
                ),
                new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(
                                        new Pose(third.getPosition(), new Angle(leftSide ? Math.PI : 0)),
                                        new Pose(second.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                                10, 2, 5, 2, 0, 1, true
                        ), scoreHeading, 6, 1,
                        0.1, 0.6, 3, 0, 0.02, true
                ),
                new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(
                                        new Pose(second.getPosition(), new Angle(leftSide ? Math.PI : 0)),
                                        new Pose(first.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                                10, 2, 5, 2, 0, 1, true
                        ), scoreHeading, 6, 1,
                        0.1, 0.6, 3, 0, 0.02, true
                )
        ).schedule();

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
        SwerveSubsystem.getInstance().setRampRate(0.5);
        OI.getInstance().setupControls();
        Odometry.getInstance().disableCustomCam();
        Odometry.getInstance().setScoringCam(false);
        OI.getInstance().zeroAll();
        StateMachine.getInstance().setIntakeOff();


//        new InitAutoCommand(new Pose(new Point(40.45 + 32, 42.19 - 20.873), new Angle(Math.PI))).schedule();

//        SwerveSubsystem.getInstance().setRampRate(0);
//        new FastOvershootBalance(3, 0.7, false).schedule();
//        new Balance(false).schedule();
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
//        OI.getInstance().getDriveController().setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
//        OI.getInstance().getDriveController().setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
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
