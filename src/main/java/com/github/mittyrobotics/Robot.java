package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePurePursuitCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotConstants;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeConstants;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
6trt * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  TrapezoidalMotionProfile tp;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    tp = new TrapezoidalMotionProfile(10, 10, 10, 0, 30, 0.05);
//    tp = new TrapezoidalMotionProfile(0.1, 1, 10);

    SwerveSubsystem.getInstance().initHardware();

    TelescopeSubsystem.getInstance().initHardware();
    PivotSubsystem.getInstance().initHardware();

    Gyro.getInstance().initHardware();

    OI.getInstance().setUpTuningControls();


    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    PivotSubsystem.getInstance().setBrakeMode();
    TelescopeSubsystem.getInstance().setBrakeMode();




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
//    CommandScheduler.getInstance().run();

    SwerveSubsystem.getInstance().updateForwardKinematics();
    SmartDashboard.putNumber("Raw vel", PivotSubsystem.getInstance().rawVel());
//    System.out.println(PivotSubsystem.getInstance().getPositionDegrees());


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
    SwervePath[] paths = {
            new SwervePath(
                    new QuinticHermiteSpline(
                            new Point(0, 0), new Angle(2.5*Math.PI/2),
                            new Point(-26/39.37, (-224+26)/39.37), new Angle(3*Math.PI/2)),
                    new Angle(0), new Angle(Math.PI),
                    0, 0, 6., 12., 1, 0.2, 0.4, 2.5, 0, 0.02, 0.5
            ),
            new SwervePath(
                    new QuinticHermiteSpline(
                            new Point(-26/39.37, (-224+26)/39.37), new Angle(Math.PI/2),
                            new Point(0, 0), new Angle(0.5*Math.PI/2)),
                    new Angle(Math.PI), new Angle(0),
                    0, 0,6., 12., 1, 0.2, 0.4, 2.5, 0, 0.02, 0.5
            )
    };

    SwervePurePursuitCommand command = new SwervePurePursuitCommand(0.05, 0.07, paths);
//    SwerveSubsystem.getInstance().setDefaultCommand(command);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

//    System.out.println("RADIANS THETA: " + PivotSubsystem.getInstance().getPositionRadians());
//    System.out.println("METERS R: " + TelescopeSubsystem.getInstance().getDistanceMeters());

    //high, mid, hp, no low yet since no bumpers
//    System.out.println("HALIFAX TOP: " + PivotSubsystem.getInstance().getHalifaxTopContact());
//    System.out.println("ENCODER DEGREES: " + PivotSubsystem.getInstance().getPositionDegrees());
//    System.out.println("MAX: " + !TelescopeSubsystem.getInstance().getHalifaxMaxContact());
//    System.out.println(TelescopeSubsystem.getInstance().getDistanceMeters());
//    System.out.println(TelescopeSubsystem.getInstance().getOutput());


    //

    //-9.033033020642339 ENCODER LEAVE

//-5.024293928730245 ENCODER FIRST
/*
    System.out.println("MAX: " + !TelescopeSubsystem.getInstance().getHalifaxMaxContact());
    System.out.println("Min: " + !TelescopeSubsystem.getInstance().getHalifaxMinContact());
    if(TelescopeSubsystem.getInstance().getHalifaxMaxContact()) {
      TelescopeSubsystem.getInstance().setMotor(-0.1);
    } else if(TelescopeSubsystem.getInstance().getHalifaxMinContact()) {
      TelescopeSubsystem.getInstance().setMotor(0.1);
    }
*/
//    switch (m_autoSelected) {
//      case kCustomAuto:
//        // Put custom auto code here
//        break;
//      case kDefaultAuto:
//      default:
//        // Put default auto code here
//        break;
//    }

    System.out.println(Gyro.getInstance().getHeadingRadians());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double vel = tp.update(0.02, PivotSubsystem.getInstance().getVelocityDegreesPerSecond(), PivotSubsystem.getInstance().getPositionDegrees());

//    PivotSubsystem.getInstance().configPID(0.0003, 0, 0.00000); // FOR RAW PID
    PivotSubsystem.getInstance().configPID(0.0001, 0, 0);
//    PivotSubsystem.getInstance().configPID(0, 0, 0);
//    PivotSubsystem.getInstance().setVelocityDegreesPerSecond(30);
//    PivotSubsystem.getInstance().setPositionRadians(Math.PI/4);
    PivotSubsystem.getInstance().setVelocityDegreesPerSecond(vel);
    SmartDashboard.putNumber("Pos Degrees", PivotSubsystem.getInstance().getPositionDegrees());
//    PivotSubsystem.getInstance().setMotor(0.2);
    //668, 1251, 1751
//    PivotSubsystem.getInstance().setRaw();
//    System.out.println(PivotSubsystem.getInstance().getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("Pivot Pos", PivotSubsystem.getInstance().rawPos());


//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setP(.003);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setReference(tp.update(0.02, TelescopeSubsystem.getInstance().getVelocityInchesPerSecond() / 600.,
//            TelescopeSubsystem.getInstance().getDistanceInches()), CANSparkMax.ControlType.kVelocity);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setReference(10/39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, CANSparkMax.ControlType.kPosition);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setP(0.001);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setI(0.000);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setD(0.000);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setFF();
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setReference((2/39.37) / TelescopeConstants.METERS_PER_MOTOR_REV / 10, CANSparkMax.ControlType.kVelocity);
//    System.out.println(TelescopeSubsystem.getInstance().getVelocityInchesPerSecond());
//    TelescopeSubsystem.getInstance().getNeo().set(0.1);
//    System.out.println(TelescopeSubsystem.getInstance().getNeo().getEncoder().getVelocity());

//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setFF(1/6000.);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setSmartMotionMaxAccel(0, 0);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setSmartMotionMaxVelocity(0, 0);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setSmartMotionMinOutputVelocity(0.05, 0);
//    TelescopeSubsystem.getInstance().getNeo().getPIDController().setSmartMotionAllowedClosedLoopError(0, 0);
//    System.out.println("Pos: " + TelescopeSubsystem.getInstance().getNeo().getEncoder().getPosition());
//    TelescopeSubsystem.getInstance().setPositionMeters(10/39.37);
    System.out.println("Percent output: " + PivotSubsystem.getInstance().getOutput());
    //4.5



//    System.out.println("RADIANS THETA: " + PivotSubsystem.getInstance().getPositionRadians());
//    System.out.println("METERS R: " + TelescopeSubsystem.getInstance().getDistanceMeters());

//    if (OI.getInstance().getOperatorController().getLeftY() < -0.1) ArmKinematics.incrementHeight(true);
//    if (OI.getInstance().getOperatorController().getLeftY() > 0.1) ArmKinematics.incrementHeight(false);
//    if (OI.getInstance().getOperatorController().getRightY() < -0.1) ArmKinematics.incrementDistance(true);
//    if (OI.getInstance().getOperatorController().getRightY() > 0.1) ArmKinematics.incrementDistance(false);




  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    TelescopeSubsystem.getInstance().setCoastMode();
    PivotSubsystem.getInstance().setCoastMode();
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
