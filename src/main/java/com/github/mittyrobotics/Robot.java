
package com.github.mittyrobotics;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePurePursuitCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.drivetrain.commands.JoystickThrottleCommand;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
//  AddressableLED led;
//  AddressableLEDBuffer buffer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
//    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
//    m_chooser.addOption("My Auto", kCustomAuto);
//    SmartDashboard.putData("Auto choices", m_chooser);
//
//    led = new AddressableLED(0);
//    buffer = new AddressableLEDBuffer(150);
//
//    led.setLength(buffer.getLength());
//    led.setData(buffer);
//    led.start();
    Gyro.getInstance().initHardware();
    SwerveSubsystem.getInstance().initHardware();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {


    CommandScheduler.getInstance().run();
//    for (int i = 0; i < buffer.getLength(); i++) {
//      buffer.setRGB(i, 255, 0, 0);
//    }
//    led.setData(buffer);
    SmartDashboard.putString("Pose", SwerveSubsystem.getInstance().getPose().toString());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

//    SwerveSubsystem.getInstance().setAnglesZero();


//    SwerveSubsystem.getInstance().setWheelPercentOutput(1);

    //Pose poseStart = new Pose(new Point(-271, 116), new Angle(Math.PI/2), false);
    int accel = 12;
    int maxSpeed = 6;
    double whenToEnd = 0.5;
    int decel = 3;
//
    SwervePath[] paths = {
//            //straight line max speed test
////            new SwervePath(
////                    new QuinticHermiteSpline(new Point(0, 0), new Angle(Math.PI/2), new Point(2, -4), new Angle(Math.PI/2)),
////                    new Angle(0),
////                    new Angle(0),
////                    0, 0, maxSpeed, accel, 0.75, 0.2, 0.2, 2.5, 0, 0.02, whenToEnd
////            ),
//
//            //simulate approximate game path
            new SwervePath(
                    new QuinticHermiteSpline(new Point(0, 0), new Angle(-Math.PI/2), new Point(2, -6), new Angle(-Math.PI/4)),
                    new Angle(0),
                    new Angle(Math.PI/2),
                    0, 0, maxSpeed, accel, decel, 0.2, 0.2, 2.5, 0, 0.02, whenToEnd
            ),

            new SwervePath(
                    new QuinticHermiteSpline(new Point(2, -6), new Angle(3*Math.PI/4), new Point(0, 0), new Angle(Math.PI/2)),
                    new Angle(Math.PI/2),
                    new Angle(0),
                    0, 0, maxSpeed, accel, decel, 0.2, 0.2, 2.5, 0, 0.02, whenToEnd
            )
//
//            //W
//
////            new SwervePath(
////                    new QuinticHermiteSpline(new Point(0,0), new Vector(2, -10), new Vector(0, 0), new Point(1.25, -1.5), new Vector(1, 0), new Vector(0, 0)),
////                    new Angle(0),
////                    new Angle(Math.PI),
////                    0, 0, maxSpeed, accel, 0.75, 0.2, 0.2, 2.5, 0, 0.02, whenToEnd),
////
////            new SwervePath(
////                    new QuinticHermiteSpline(new Point(1.25,-1.5), new Vector(1, 0), new Vector(0, 0), new Point(2.5, 0), new Vector(2, 10), new Vector(0, 0)),
////                    new Angle(Math.PI),
////                    new Angle(0),
////                    0, 0, maxSpeed, accel, 0.75, 0.2, 0.2, 2.5, 0, 0.02, whenToEnd),
////
////            new SwervePath(
////                    new QuinticHermiteSpline(new Point(2.5,0), new Vector(-2, -10), new Vector(0, 0), new Point(1.25, -1.5), new Vector(-1, 0), new Vector(0, 0)),
////                    new Angle(0),
////                    new Angle(Math.PI),
////                    0, 0, maxSpeed, accel, 0.75, 0.2, 0.2, 2.5, 0, 0.02, whenToEnd),
////
////            new SwervePath(
////                    new QuinticHermiteSpline(new Point(1.25,-1.5), new Vector(-1, 0), new Vector(0, 0), new Point(0, 0), new Vector(-2, 10), new Vector(0, 0)),
////                    new Angle(Math.PI),
////                    new Angle(0),
////                    0, 0, maxSpeed, accel, 0.75, 0.2, 0.2, 2.5, 0, 0.02, whenToEnd),
//
////
//
////                new SwervePath(
////                        new QuinticHermiteSpline(new Point(0,0), new Vector(-5, 0), new Vector(0, 0), new Point(0, 1.5), new Vector(-5, 0), new Vector(0, 0)),
////                        new Angle(0),
////                        new Angle(Math.PI/2),
////                        0, 0, 1.5, 0.5, 0.5, 0.2, 2.5, 0, 0.02),
////
////                new SwervePath(
////                        new QuinticHermiteSpline(new Point(0,1.5), new Vector(-5, 0), new Vector(0, 0), new Point(0, 0), new Vector(-5, 0), new Vector(0, 0)),
////                        new Angle(Math.PI/2),
////                        new Angle(2 * Math.PI),
////                        0, 0, 1.5, 0.5, 0.5, 0.2, 2.5, 0, 0.02),
//
//            //CIRCLE PATH
////                new SwervePath(
////                        new QuinticHermiteSpline(new Point(0,0), new Angle(0), new Point(0, 2), new Angle(Math.PI)),
////                        new Angle(0),
////                        new Angle(Math.PI),
////                        0, 0, 1, 0.5, 0.5, 0.2, 2.5, 0, 0.02),
////
//            //S PATH
////                new SwervePath(
////                        new QuinticHermiteSpline(new Point(0,0), new Angle(0), new Point(0, 2), new Angle(0)),
////                        new Angle(0),
////                        new Angle(Math.PI),
////                        0, 0, 1, 0.6, 0.6, 0.2, 2.5, 0, 0.02),
////
//            // D PATH
////                new SwervePath(
////                        new QuinticHermiteSpline(new Point(0,0), new Angle(Math.PI/2), new Point(0, 2.5), new Angle(Math.PI/2.)),
////                        new Angle(0),
////                        new Angle(Math.PI/2),
////                        0, 0, 0.5, 0.5, 0.5, 1),
////
////                new SwervePath(
////                        new QuinticHermiteSpline(new Point(0,2.5), new Angle(Math.PI), new Point(0, 0), new Angle(0)),
////                        new Angle(Math.PI/2),
////                        new Angle(0),
////                        0, 0, 0.5, 0.5, 0.5, 1)
//
////                new SwervePath(
////                        new QuinticHermiteSpline(new Point(0,2), new Angle(Math.PI), new Point(0, 0), new Angle(0)),
////                        new Angle(0),
////                        new Angle(0),
////                        0, 0, 0.5, 0.5, 0.5, 1)
//
////                new SwervePath(
////                        new QuinticHermiteSpline(new Point(5,5), new Angle(Math.atan2(2, 3)), new Point(8,7), new Angle(Math.atan2(2, 3))),
////                        new Angle(0),
////                        new Angle(0)),
////                new SwervePath(
////                        new QuinticHermiteSpline(new Point(8,7), new Angle(Math.PI), new Point(2,2), new Angle(Math.PI)),
////                        new Angle(0),
////                        new Angle(0)),
    };
//was 0.05, 0.07
    SwervePurePursuitCommand command = new SwervePurePursuitCommand(0.05, 0.07, paths);
    SwerveSubsystem.getInstance().setDefaultCommand(command);
//
//    m_autoSelected = m_chooser.getSelected();
//    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
////    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
//    SwerveSubsystem.getInstance().setMeterPerSecond(-1);
//    SwerveSubsystem.getInstance().setWheelPercentOutput(0.3);

//    if(SwerveSubsystem.getInstance().ticks(0) > -2048 - 4096) {
//      SwerveSubsystem.getInstance().setWheelPercentOutput(-0.1);
//    } else {
//      SwerveSubsystem.getInstance().setWheelPercentOutput(0);
//    }
//    for (int i = 0; i < 4; i++) {
//      System.out.println("Velocity " + i + " " + SwerveSubsystem.getInstance().getAllSpeedsMeters()[i]);
//    }

//    System.out.println(SwerveConstants.FALCON_TICKS / SwerveConstants.DRIVE_WHEEL_TO_FALCON_GEAR_RATIO);

//    System.out.println("Wheel Zero Ticks: " + SwerveSubsystem.getInstance().ticks(0));



//    System.out.println("POSE: " + SwerveSubsystem.getInstance().getPose());
    SwerveSubsystem.getInstance().updateForwardKinematics();
//    switch (m_autoSelected) {
//      case kCustomAuto:
//        // Put custom auto code here
//        break;
//      case kDefaultAuto:
//      default:
//        // Put default auto code here
//        break;
//    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().schedule(new JoystickThrottleCommand());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*
    SwerveSubsystem.getInstance().setSwerveModule(new Vector(1, 0.0), -2);
    SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
    SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
    */
//    System.out.println(SwerveSubsystem.getInstance().getPose());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
//    SwerveSubsystem.getInstance().setAllControlMode(NeutralMode.Coast);
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
