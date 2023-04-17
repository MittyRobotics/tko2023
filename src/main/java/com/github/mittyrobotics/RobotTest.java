package com.github.mittyrobotics;

import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotTest extends Robot {
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    CANSparkMax spark;
    TrapezoidalMotionProfile tp;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        spark = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
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

//        SwerveSubsystem.getInstance().updateForwardKinematics();

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
//        TelescopeSubsystem.getInstance().setPID(0.01, 0, 0);
//        TelescopeSubsystem.getInstance().setPositionInches(10);
    /*
    TelescopeSubsystem.getInstance().setMotor(0.1);
*/

        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);

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
        switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
                break;
            case kDefaultAuto:
            default:
                // Put default auto code here
                break;
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
//        tp = new TrapezoidalMotionProfile(1, 10, 10);
//        spark.restoreFactoryDefaults();
//        spark.getEncoder().setPosition(0);
//        spark.getPIDController().setP(0.1);
//        spark.getPIDController().setI(0.0);
//        spark.getPIDController().setP(0.0);
//        spark.getPIDController().setFF(0.1);

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

//        spark.set(1);

//        System.out.println(tp.getMaxVelFromPos(9.9));
//        if (Math.abs(spark.getEncoder().getPosition() - 10) > 0.1) {
//            double output = tp.update(0.02, spark.getEncoder().getPosition());
//        System.out.println(output + " | " + spark.getEncoder().getVelocity() + " | " + spark.getEncoder().getPosition());
//
//            spark.getPIDController().setReference(output, CANSparkMax.ControlType.kVelocity);
//        } else {
//            spark.getPIDController().setReference(0, CANSparkMax.ControlType.kVelocity);
//        }
//        spark.getPIDController().setReference(0.1, CANSparkMax.ControlType.kVelocity);

//        System.out.println("RADIANS THE  TA: " + PivotSubsystem.getInstance().getPositionRadians());
//        System.out.println("METERS R: " + TelescopeSubsystem.getInstance().getDistanceMeters());

//    if (OI.getInstance().getOperatorController().getLeftY() < -0.1) ArmKinematics.incrementHeight(true);
//    if (OI.getInstance().getOperatorController().getLeftY() > 0.1) ArmKinematics.incrementHeight(false);
//    if (OI.getInstance().getOperatorController().getRightY() < -0.1) ArmKinematics.incrementDistance(true);
//    if (OI.getInstance().getOperatorController().getRightY() > 0.1) ArmKinematics.incrementDistance(false);




    }
    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
//        TelescopeSubsystem.getInstance().setCoastMode();
//        PivotSubsystem.getInstance().setCoastMode();
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
