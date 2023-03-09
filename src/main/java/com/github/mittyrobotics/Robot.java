package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.drivetrain.Pair;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.json.JSONException;

import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
6trt * each mode, as described in the TimedRobot documentation. If you change the name of this class or
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

    OI.getInstance().setupControls();


    // TODO: check this
    Odometry.getInstance().FIELD_LEFT_SIDE = false;
    Gyro.getInstance().setAngleOffset(Odometry.getInstance().FIELD_LEFT_SIDE ? Math.PI : 0);

    SwerveSubsystem.getInstance().setPose(new Pose(new Point(0, 0), new Angle(Odometry.getInstance().FIELD_LEFT_SIDE ? Math.PI : 0)));
    Odometry.getInstance().setState(0, 0, Odometry.getInstance().FIELD_LEFT_SIDE ? Math.PI : 0);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SwerveSubsystem.getInstance().updateForwardKinematics();
    Odometry.getInstance().update();

    LoggerInterface.getInstance().put("Heading", Gyro.getInstance().getHeadingRadians());
    LoggerInterface.getInstance().put("Pose", Arrays.toString(Odometry.getInstance().getPose()));
    System.out.println(Arrays.toString(Odometry.getInstance().getPose()));
    System.out.println(Gyro.getInstance().getHeadingRadians());
//    LoggerInterface.getInstance().put("Intake State", StateMachine.getInstance().getIntakingState());
//    LoggerInterface.getInstance().put("Robot State", StateMachine.getInstance().getCurrentRobotState());
//    System.out.println("mode:" + StateMachine.getInstance().getCurrentPieceState());
//    Odometry.getInstance().update();
//    SwerveSubsystem.getInstance().updateForwardKinematics();
//    SmartDashboard.putString("pose", SwerveSubsystem.getInstance().getPose().toString());
  }

  @Override
  public void autonomousInit() {

    SwerveSubsystem.getInstance().resetPose();
//
//    Odometry.getInstance().setState(162, 75, Math.PI);
//
//    SwervePath[] paths = {
//            new SwervePath(
//                    new QuinticHermiteSpline(
//                            new Point(0, 0), new Angle(3*Math.PI/2),
//                            new Point(-15/39.37, (-224+25)/39.37), new Angle(3*Math.PI/2)),
//                    new Angle(0), new Angle(Math.PI),
//                    0, 0, 6., 8., 3, 0.2, 0.4, 2.5, 0, 0.02, 0.3
//            ),
//            new SwervePath(
//                    new QuinticHermiteSpline(
//                            new Point(-15/39.37, (-224+25)/39.37), new Angle(Math.PI/2),
//                            new Point(0, 0), new Angle(Math.PI/2)),
//                    new Angle(Math.PI), new Angle(0),
//                    0, 0, 6., 8., 3, 0.2, 0.4, 2.5, 0, 0.02, 0.3
//            )
//    };
//
//    SwervePurePursuitCommand command = new SwervePurePursuitCommand(0.05, 0.07, paths);
//    SwerveSubsystem.getInstance().setDefaultCommand(command);

//    SwervePath path = new SwervePath(
//            new QuinticHermiteSpline(new Point(0, 0), new Angle(Math.PI/2), new Point(1, 1), new Angle(Math.PI/2)),
//            new Angle(0), new Angle(0),
//            0, 0, 3., 6., 3, 0.0, 0.2, 0.0, 0, 0.00, 0.5
//    );
//
//    SwerveAutoPickupCommandv1 command = new SwerveAutoPickupCommandv1(0.07, 0.05, path);
//    SwerveSubsystem.getInstance().setDefaultCommand(command);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

//    Odometry.getInstance().update();
//    System.out.println("RAD: " + PivotSubsystem.getInstance().getPositionRadians());
//    System.out.println("EXT: " + TelescopeSubsystem.getInstance().getDistanceMeters());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
//    SwerveAutoPickupCommandv2 command = new SwerveAutoPickupCommandv2(0.05, 0.05, path);
//    SwerveSubsystem.getInstance().setDefaultCommand(command);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
//    SmartDashboard.putString("pose", SwerveSubsystem.getInstance().getPose().toString());
//    try {
//      System.out.println(Arrays.toString(ArmKinematics.getVectorToGamePiece(true, 0)));
//    } catch (JSONException e) {

//    }
    System.out.println("ANGLE: " + PivotSubsystem.getInstance().getPositionRadians());
    System.out.println("RADIUS: "  + TelescopeSubsystem.getInstance().getDistanceMeters());
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    LedSubsystem.getInstance().turnOff();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
