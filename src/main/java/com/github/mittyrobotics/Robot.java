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

        Odometry.getInstance().FIELD_LEFT_SIDE = true;
        double x = 0;
        double y = 0;
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

        Odometry.getInstance().update();

        LoggerInterface.getInstance().put("Heading", Gyro.getInstance().getHeadingRadians());
        LoggerInterface.getInstance().put("Pose", Odometry.getInstance().getState());

        LoggerInterface.getInstance().put("Pose X", Odometry.getInstance().getPose()[0]);
        LoggerInterface.getInstance().put("Pose Y", Odometry.getInstance().getPose()[1]);

        CommandScheduler.getInstance().run();

    }

    @Override
    public void autonomousInit() {
        Odometry.getInstance().FIELD_LEFT_SIDE = LoggerInterface.getInstance().getValue("fieldside").equals("left");

        String auto = LoggerInterface.getInstance().getValue("auto");
        boolean balance = LoggerInterface.getInstance().getValue("autobalance").equals("true");

        switch(auto) {
            case "preload":
                int tag = Integer.parseInt(LoggerInterface.getInstance().getValue("autotag"));
                int index = Integer.parseInt(LoggerInterface.getInstance().getValue("autoindex"));
                new PreloadAndBalanceAuto(Odometry.getInstance().FIELD_LEFT_SIDE, tag, index, balance).schedule();
                break;
            case "one":
                new PlusOneConeAuto(Odometry.getInstance().FIELD_LEFT_SIDE, balance).schedule();
                break;
        }
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
//        SwerveSubsystem.getInstance().setRampRate(0.5);
//        OI.getInstance().setupControls();
//        Odometry.getInstance().disableCustomCam();
//        Odometry.getInstance().setScoringCam(false);
//        OI.getInstance().zeroAll();
//        StateMachine.getInstance().setIntakeOff();


//        new InitAutoCommand(new Pose(new Point(40.45 + 32, 42.19 - 20.873), new Angle(Math.PI))).schedule();

        SwerveSubsystem.getInstance().setRampRate(0);
//        new FastOvershootBalance(3, 0.7, false).schedule();
//        new TimedOvershootBalance(3, 1000, 0.7, false).schedule();
        new TimedBangBangBalance(3, 1000, 1, false).schedule();

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        System.out.println(Gyro.getInstance().getHeadingRadians() + "  " + Gyro.getInstance().getHeadingRadiansNoOffset());
//    SmartDashboard.putString("pose", SwerveSubsystem.getInstance().getPose().toString());
//    try {
//      System.out.println(Arrays.toString(ArmKinematics.getVectorToGamePiece(true, 0)));
//    } catch (JSONException e) {

//    }

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

        SwerveSubsystem.getInstance().setAnglesZero();
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
        System.out.println(LoggerInterface.getInstance().getValue("fieldside"));
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
