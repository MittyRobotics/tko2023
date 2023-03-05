package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.intake.IntakeSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj.TimedRobot;
import org.json.JSONException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
6trt * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class RobotFuckTesting extends TimedRobot {
  @Override
  public void robotInit() {
    Gyro.getInstance().initHardware();
    SwerveSubsystem.getInstance().initHardware();
  }

  @Override
  public void robotPeriodic() {
//    CommandScheduler.getInstance().run();
//    StateMachine.getInstance().update();
//    System.out.println("mode:" + StateMachine.getInstance().getCurrentPieceState());

//    Odometry.getInstance().update();
//
//
//    SmartDashboard.putString("pose", SwerveSubsystem.getInstance().getPose().toString());
  }

  @Override
  public void autonomousInit() {

    SwerveSubsystem.getInstance().resetPose();

    Odometry.getInstance().setState(0, 0, Math.PI);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SwerveSubsystem.getInstance().updateForwardKinematics();

//    System.out.println(SwerveSubsystem.getInstance().forwardKinematics.getLatestPose());

    Odometry.getInstance().update();
//    System.out.println("RAD: " + PivotSubsystem.getInstance().getPositionRadians());
//    System.out.println("EXT: " + TelescopeSubsystem.getInstance().getDistanceMeters());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
//      TelescopeSubsystem.getInstance().getNeo().getEncoder().setPosition(0);
//      SwerveSubsystem.getInstance().setDefaultCommand(new JoystickThrottleCommand());
//    SwervePath path = new SwervePath(
//            new QuinticHermiteSpline(new Point(0, 0), new Angle(Math.PI/2), new Point(0.5, 0.5), new Angle(Math.PI/2)),
//            new Angle(0), new Angle(0),
//            0, 0, 3., 12., 3, 0.0, 0.2, 0.0, 0, 0.00, 0.5
//    );
//
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
//    System.out.println("ANGLE: " + PivotSubsystem.getInstance().getPositionRadians());
//    System.out.println("RADIUS: "  + TelescopeSubsystem.getInstance().getDistanceMeters());
//    ClawRollerSubsystem.getInstance().setRoller(-0.4);  //NEGATIVE IS ROLLER INWARDS

//    ClawGrabberSubsystem.getInstance().setMotor(0.1);

//    SmartDashboard.putNumber("Neo Current", ClawRollerSubsystem.getInstance().getCurrent());

//    System.out.println(ClawGrabberSubsystem.getInstance().getProximitySensor());
//    System.out.println(ClawGrabberSubsystem.getInstance().getEncoderValue());

  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

//    LedSubsystem.getInstance().turnOff();
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