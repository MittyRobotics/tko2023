package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.arm.AutoScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.SwerveAutoDriveToScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.SwerveAutoScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePurePursuitCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.drivetrain.Pair;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.drivetrain.commands.JoystickThrottleCommand;
import com.github.mittyrobotics.intake.IntakeSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.json.JSONException;

import java.util.Arrays;

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

//    OI.getInstance().setupControls();


        // TODO: check this
        Odometry.getInstance().FIELD_LEFT_SIDE = true;

        double x = 0;
        double y = 0;
        double t = Math.PI;

//        Gyro.getInstance().setAngleOffset(Odometry.getInstance().FIELD_LEFT_SIDE ? Math.PI : 0);
//
//        SwerveSubsystem.getInstance().setPose(new Pose(new Point(0, 0), new Angle(Odometry.getInstance().FIELD_LEFT_SIDE ? Math.PI : 0)));
//        Odometry.getInstance().setState(0, 0, Odometry.getInstance().FIELD_LEFT_SIDE ? Math.PI : 0);
        Gyro.getInstance().setAngleOffset(t);
        Odometry.getInstance().setState(x, y, t);
        SwerveSubsystem.getInstance().setPose(new Pose(new Point(0, 0),
                new Angle(Gyro.getInstance().getHeadingRadians())));

        Odometry.getInstance().setScoringCam(true);



    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

//        System.out.println(LoggerInterface.getInstance().getGamePiece());
        SwerveSubsystem.getInstance().updateForwardKinematics();
//        System.out.println(SwerveSubsystem.getInstance().forwardKinematics.getLatestTime());
//        System.out.println(SwerveSubsystem.getInstance().getPose());
//        Odometry.getInstance().update();

//        System.out.println(OI.getInstance().getDriveController().getRightTriggerAxis());

//    LoggerInterface.getInstance().put("Heading", Gyro.getInstance().getHeadingRadians());
//    LoggerInterface.getInstance().put("Pose", Arrays.toString(Odometry.getInstance().getPose()));
        System.out.println(Odometry.getInstance().getState());
//        System.out.println(Odometry.getInstance().getClosestScoringZone()[1]);
//    System.out.println(Gyro.getInstance().getHeadingRadians());

//    System.out.println(IntakeSubsystem.getInstance().proxSensorTrigger());
//    LoggerInterface.getInstance().put("Intake State", StateMachine.getInstance().getIntakingState());
//    LoggerInterface.getInstance().put("Robot State", StateMachine.getInstance().getCurrentRobotState());
//    System.out.println("mode:" + StateMachine.getInstance().getCurrentPieceState());
//    System.out.println("FROM SWERVE: " + SwerveSubsystem.getInstance().getPose());
//    System.out.println("FROM ODOMETRY: " + Odometry.getInstance().getState());
//    SmartDashboard.putString("pose", SwerveSubsystem.getInstance().getPose().toString());
    }

    @Override
    public void autonomousInit() {

//    SwerveSubsystem.getInstance().resetPose();
//
//    Odometry.getInstance().setState(103, 6, Math.PI);

        Pose init = Odometry.getInstance().getState();

        SwervePath[] paths = {
            new SwervePath(
                    new QuinticHermiteSpline(init, new Pose(Point.add(init.getPosition(), new Point(-2 * 39.37, 1 * 39.37)), init.getHeading())),
                    new Angle(Math.PI),
                    new Angle(0),
                    0, 0, 0.5, 0.5, 0.5,
                    0.2, 15, 2.5, 0, 0.02, 0.6
            )
        };

        SwervePurePursuitCommand command = new SwervePurePursuitCommand(5, 0.05, paths);
        CommandScheduler.getInstance().schedule(command);

//        StateMachine.getInstance().setIntakeStowing();
//        CommandScheduler.getInstance().schedule(
//                new AutoScoreCommand(Odometry.getInstance().getClosestScoringZone()[2], StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE)
//        );

//    SwervePath path = new SwervePath(
//            new QuinticHermiteSpline(new Point(0, 0), new Angle(Math.PI/2), new Point(1, 1), new Angle(Math.PI/2)),
//            new Angle(0), new Angle(0),
//            0, 0, 3., 6., 3, 0.0, 0.2, 0.0, 0, 0.00, 0.5
//    );
//
//    SwerveAutoPickupCommandv1 command = new SwerveAutoPickupCommandv1(0.07, 0.05, path);
//    SwerveSubsystem.getInstance().setDefaultCommand(command);
//    StateMachine.getInstance().setIntakeStowing();
//    CommandScheduler.getInstance().schedule(new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CUBE, false));
//    CommandScheduler.getInstance().schedule(new AutoScoreCommand(new Pose(new Point(5, 0), new Angle(Odometry.getInstance().FIELD_LEFT_SIDE ? Math.PI : 0)),
//            StateMachine.RobotState.HIGH, true));

//        SwervePath path = new SwervePath(
//                new QuinticHermiteSpline(
//                        Odometry.getInstance().getState(),
//                        new Pose(new Point(30, 30), new Angle(Math.PI))
////                        new Pose(Point.add(Odometry.getInstance().getState().getPosition(), new Point(24, 0)), new Angle(Math.PI))
//                ),
//                new Angle(0), new Angle(0),
//                0, 0, 6., 8., 3, 0.2, 0.4, 2.5, 0, 0.02, 0.3
//        );
//
//        SwerveAutoDriveToScoreCommand command = new SwerveAutoDriveToScoreCommand(0.05, 0.07, path);
////        System.out.println(path.getSpline().getLength(1.0, 17));
//        SwerveSubsystem.getInstance().setDefaultCommand(command);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

//    Odometry.getInstance().update();
//    System.out.println("RAD: " + PivotSubsystem.getInstance().getPositionRadians());
//    System.out.println("EXT: " + TelescopeSubsystem.getInstance().getDistanceMeters());
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
    OI.getInstance().setupControls();
//    SwerveAutoPickupCommandv2 command = new SwerveAutoPickupCommandv2(0.05, 0.05, path);
//    SwerveSubsystem.getInstance().setDefaultCommand(command);
//    SwerveSubsystem.getInstance().setDefaultCommand(new JoystickThrottleCommand());
//
//    CommandScheduler.getInstance().schedule(new SwerveAutoScoreCommand(
//            new Pose(
//                    Point.add(Odometry.getInstance().getScoringZone(6)[0].getPosition(), new Point(32, 0)),
//                    new Angle(Odometry.getInstance().FIELD_LEFT_SIDE ? Math.PI : 0)
//            ))
//    );
//
//        StateMachine.getInstance().setIntakeStowing();
//
//    CommandScheduler.getInstance().schedule(new AutoScoreCommand(
//            new Pose(
//            Point.add(Odometry.getInstance().getScoringZone(6)[1].getPosition(), new Point(32, 0)),
//            new Angle(Odometry.getInstance().FIELD_LEFT_SIDE ? Math.PI : 0)
//    ), StateMachine.RobotState.MID, StateMachine.PieceState.CUBE, false
//            ));

//        SwerveSubsystem.getInstance().setPose(new Pose(new Point(0, 0), new Angle(Math.PI)));
//
//        SwerveSubsystem.getInstance().updateForwardKinematics();
//        Odometry.getInstance().update();


//        CommandScheduler.getInstance().schedule(new SwerveAutoScoreCommand(
//                new Pose(
//                        new Point(-80, 20),
//                        new Angle(Math.PI)
//                ))
//        );

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
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
        TelescopeSubsystem.getInstance().setCoastMode();
        PivotSubsystem.getInstance().setCoastMode();

        LedSubsystem.getInstance().turnOff();
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
