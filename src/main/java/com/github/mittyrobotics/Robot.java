package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePurePursuitCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.pivot.PivotConstants;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeConstants;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
6trt * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  TrapezoidalMotionProfile tpPivot, tpTelescope;
  double lastTime;

  @Override
  public void robotInit() {
    // WORKS FOR PIVOT
    tpPivot = new TrapezoidalMotionProfile(360 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 360 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 360 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 20 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3);

    //WORKS FOR TELESCOPE
    tpTelescope = new TrapezoidalMotionProfile(10 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 10 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 0.2);


    SwerveSubsystem.getInstance().initHardware();
    TelescopeSubsystem.getInstance().initHardware();
    PivotSubsystem.getInstance().initHardware();
    Gyro.getInstance().initHardware();
    OI.getInstance().setUpTuningControls();
    PivotSubsystem.getInstance().setBrakeMode();
    TelescopeSubsystem.getInstance().setBrakeMode();

  }

  @Override
  public void robotPeriodic() {
//    CommandScheduler.getInstance().run();

    SwerveSubsystem.getInstance().updateForwardKinematics();
//    SmartDashboard.putNumber("Raw vel", PivotSubsystem.getInstance().rawVel());
//    System.out.println(PivotSubsystem.getInstance().getPositionDegrees());

  }

  @Override
  public void autonomousInit() {
    SwerveSubsystem.getInstance().resetPose();

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
//    System.out.println(Gyro.getInstance().getHeadingRadians());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
      TelescopeSubsystem.getInstance().getNeo().getEncoder().setPosition(0);
      lastTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(OI.getInstance().getOperatorController().getRightBumper()) {
      if (OI.getInstance().getOperatorController().getAButton()) {
        tpPivot.changeSetpoint(45 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, PivotSubsystem.getInstance().rawPos(), PivotSubsystem.getInstance().rawVel() / 60);
      } else if (OI.getInstance().getOperatorController().getBButton()) {
        tpPivot.changeSetpoint(0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, PivotSubsystem.getInstance().rawPos(), PivotSubsystem.getInstance().rawVel() / 60);
      } else if (OI.getInstance().getOperatorController().getXButton()) {
        tpPivot.changeSetpoint(90 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, PivotSubsystem.getInstance().rawPos(), PivotSubsystem.getInstance().rawVel() / 60);
      }
    } else if (OI.getInstance().getOperatorController().getLeftBumper()) {
      if (OI.getInstance().getOperatorController().getAButton()) {
        tpTelescope.changeSetpoint(12 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, TelescopeSubsystem.getInstance().rawPos(), TelescopeSubsystem.getInstance().rawVel() / 60);
      } else if (OI.getInstance().getOperatorController().getBButton()) {
        tpTelescope.changeSetpoint(0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, TelescopeSubsystem.getInstance().rawPos(), TelescopeSubsystem.getInstance().rawVel() / 60);
      } else if (OI.getInstance().getOperatorController().getXButton()) {
        tpTelescope.changeSetpoint(24 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, TelescopeSubsystem.getInstance().rawPos(), TelescopeSubsystem.getInstance().rawVel() / 60);
      }
    }
//    double pivotSetPoint = 45 - OI.getInstance().getOperatorController().getLeftY() * 45;
//    tpTelescope.setDecel(20 + Math.sin(PivotSubsystem.getInstance().getPositionRadians()) * (60 - 20));
//    System.out.println("PIVOT SP: " + pivotSetPoint / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO);
//    System.out.println("DECEL: " + (20 + Math.sin(PivotSubsystem.getInstance().getPositionRadians()) * (60 - 20)));

    double rightX = (OI.getInstance().getOperatorController().getRightX() + 1)/2;
//    double telescopeSetPoint = rightX * 24 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV;
//    double telescopeSetPoint = 24 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV;
//    tpTelescope.setSetpoint(telescopeSetPoint);


    boolean movingDown = tpPivot.getSetpoint() > PivotSubsystem.getInstance().rawPos();

    double tuningConstant = 0;
    double pivotFF = 0.3/(1765. + tuningConstant * TelescopeSubsystem.getInstance().getDistanceInches()) * (movingDown ? 1 : 1);

    double pidmax = 0.0002;

    double telescopeP = Math.max(0.0001, pidmax - (pidmax - 0.0001) * Math.sin(PivotSubsystem.getInstance().getPositionRadians()));
    double telescopeFF = 0.25 / (600 + (900 - 600) * Math.sin(PivotSubsystem.getInstance().getPositionRadians())); //(PivotSubsystem.getInstance().getPositionDegrees() / 90));
//    System.out.println("telescopeP: " + telescopeP);
//    System.out.println("telescopeFF: " + 0.2/telescopeFF);
    // FOR HORIZONTAL
//    TelescopeSubsystem.getInstance().setPID(0.0003, 0, 0);
//    TelescopeSubsystem.getInstance().setPID(telescopeP, 0, 0);


//    TelescopeSubsystem.getInstance().setPID(telescopeP <= pidmax ? telescopeP : 0, 0, 0);
    TelescopeSubsystem.getInstance().setPID(0, 0,0);
    TelescopeSubsystem.getInstance().getNeo().getPIDController().setFF(telescopeFF);

    PivotSubsystem.getInstance().setFF(pivotFF);
//    PivotSubsystem.getInstance().configPID(0, 0,0);
    PivotSubsystem.getInstance().configPID(0.000025, 0, 0);


//    PivotSubsystem.getInstance().configPID(0.0003, 0, 0.00000); // FOR RAW PID
//    PivotSubsystem.getInstance().configPID(0.0003 + 0.00022 * 24/14., 0, 0.00000); // FOR RAW PID

    double velTelescope = 0;
    if(OI.getInstance().getOperatorController().getLeftTriggerAxis() > 0.2)
        velTelescope = 60 * tpTelescope.update(Timer.getFPGATimestamp() - lastTime, TelescopeSubsystem.getInstance().rawPos());
    double velPivot = 0;
    if(OI.getInstance().getOperatorController().getRightTriggerAxis() > 0.2)
        velPivot = 60 * tpPivot.update(Timer.getFPGATimestamp() - lastTime, PivotSubsystem.getInstance().rawPos());
    lastTime = Timer.getFPGATimestamp();
//    PivotSubsystem.getInstance().configPID(0.001, 0, 0);
//    System.out.println(TelescopeSubsystem.getInstance().getHalifaxMaxContact());
//    TelescopeSubsystem.getInstance().setMotor(TelescopeSubsystem.getInstance().getHalifaxMaxContact() ? 0 : 0.1);
    //450
    SmartDashboard.putNumber("Telescope Pos", TelescopeSubsystem.getInstance().getDistanceInches());
    SmartDashboard.putNumber("Telescope Vel", TelescopeSubsystem.getInstance().rawVel());
//    PivotSubsystem.getInstance().setVelocityDegreesPerSecond(30);
//    PivotSubsystem.getInstance().setPositionRadians(Math.PI/4);
//    PivotSubsystem.getInstance().setVelocityDegreesPerSecond(vel);
    SmartDashboard.putNumber("Pos Degrees", PivotSubsystem.getInstance().getPositionDegrees());
    SmartDashboard.putNumber("Pivot Vel", velPivot);
    SmartDashboard.putNumber("Pivot Raw Vel", PivotSubsystem.getInstance().rawVel());

//    TelescopeSubsystem.getInstance().setMotor(0.05);
    //668, 1251, 1751
    TelescopeSubsystem.getInstance().setRaw(velTelescope);
    PivotSubsystem.getInstance().setRaw(velPivot);
//    PivotSubsystem.getInstance().setMotor(0.1);
    //
//    System.out.println(PivotSubsystem.getInstance().getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("Pivot raw vel", PivotSubsystem.getInstance().rawVel());


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
//    System.out.println("Percent output: " + TelescopeSubsystem.getInstance().getOutput());
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
