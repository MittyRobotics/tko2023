package com.github.mittyrobotics;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  public static CANSparkMax armAngleSpark1, armAngleSpark2;

  public static CANSparkMax extensionSpark;

  public static CANSparkMax gripSpark1, gripSpark2;

  public static Encoder armAngleEncoder1, armAngleEncoder2;
  public static Encoder extensionEncoder;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);




  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() { }


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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
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
    armAngleSpark1 = new CANSparkMax(Constants.Arm_Spark_IDs[0], CANSparkMaxLowLevel.MotorType.kBrushless);
    armAngleSpark2 = new CANSparkMax(Constants.Arm_Spark_IDs[1], CANSparkMaxLowLevel.MotorType.kBrushless);

    extensionSpark = new CANSparkMax(Constants.Arm_Spark_IDs[2], CANSparkMaxLowLevel.MotorType.kBrushless);

    gripSpark1 = new CANSparkMax(Constants.Arm_Spark_IDs[3], CANSparkMaxLowLevel.MotorType.kBrushless);
    gripSpark2 = new CANSparkMax(Constants.Arm_Spark_IDs[4], CANSparkMaxLowLevel.MotorType.kBrushless);

    armAngleEncoder1 = new Encoder(Constants.ARM_ENCODER_IDS[0],Constants.ARM_ENCODER_IDS[1]);
    //Find out why there are two parameters for encoder

    armAngleEncoder1.setDistancePerPulse(1/Constants.ROTATIONS_PER_DEGREE);

    extensionEncoder = new Encoder(1,2);



  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (armAngleEncoder1.getDistance()<45*Constants.ROTATIONS_PER_DEGREE){
      armAngleSpark1.set(1);
      armAngleSpark2.set(1);

    }
    if(extensionEncoder.getDistance()<Constants.MAX_EXTENSION){
      extensionSpark.set(1);
    }
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

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
