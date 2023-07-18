<<<<<<< Updated upstream

package com.github.mittyrobotics;

import com.github.mittyrobotics.util.TrapezoidalProfile;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
=======
/*
 * MIT License
 *
 * Copyright (c) 2021 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.github.mittyrobotics.OI;
import com.github.mittyrobotics.commands.TankDriveCommand;

import com.github.mittyrobotics.subsystems.DriveTrainSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

<<<<<<< Updated upstream
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  TrapezoidalProfile tp;
  CANSparkMax spark;
  double lastTime;
//  DigitalInput extensionMin;
//  DigitalInput pivotMax;
//  DigitalInput pivotMin;


  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    tp = new TrapezoidalProfile(1000, 1000, 3000, 0, 1000, 0);
    spark = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    spark.getPIDController().setFF(1/5000.);
    spark.getPIDController().setP(0.0001);
    spark.getPIDController().setI(0.);
    spark.getPIDController().setD(0.);



//    extensionMin
//    DigitalInput pivotMax
//    DigitalInput pivotMin

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
  public void robotPeriodic() {
//    System.out.println("EXTENSION: " +
//            "" + extensionMax.get());
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

    System.out.println(spark.getEncoder().getPosition());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    spark.getEncoder().setPosition(0);
    lastTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double output = tp.update(Timer.getFPGATimestamp() - lastTime, spark.getEncoder().getPosition());
    spark.getPIDController().setReference(output, CANSparkMax.ControlType.kVelocity);
//    spark.getPIDController().setReference(3000, CANSparkMax.ControlType.kVelocity);
//    System.out.println(spark.getAppliedOutput());
    SmartDashboard.putNumber("VEL", spark.getEncoder().getVelocity());
    SmartDashboard.putNumber("POS", spark.getEncoder().getPosition());
    SmartDashboard.putNumber("OUTPUT", output);
    lastTime = Timer.getFPGATimestamp();
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
=======
public class Robot extends TimedRobot {
    /**************************
     *** Used random IDs :) ***
     **************************/

    CANSparkMax sparkLeft, sparkRight;

 //   WPI_TalonSRX left1, right1;

    @Override
    public void robotInit() {
        DriveTrainSubsystem.getInstance().initHardware();
        DriveTrainSubsystem.getInstance().setDefaultCommand(new TankDriveCommand());
        OI.getInstance().initOI();;

        sparkRight  = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        sparkLeft  = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

        sparkLeft.setInverted(false);
        sparkRight.setInverted(false);




    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {

      //  sparkLeft.set(0.5);
       // sparkRight.set(0.5);
     //   left1.set(0.5);
    }

    @Override
    public void teleopPeriodic() {
        DriveTrainSubsystem.getInstance().setDefaultCommand(new TankDriveCommand());
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
>>>>>>> Stashed changes
