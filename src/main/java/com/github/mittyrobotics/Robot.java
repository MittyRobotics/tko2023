package com.github.mittyrobotics;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.*;



public class Robot extends TimedRobot {

    WPI_TalonSRX motor_rightf, motor_leftf;
    CANSparkMax motor_rightb, motor_leftb;

    @Override
    public void robotInit() {
        //IntakeSystem.getInstance();
        //IntakeSystem.getInstance().initHardware();
        //OutakeSystem.getInstance();
        //OutakeSystem.getInstance().initHardware();
        //OI.getInstance();
        DriveTrainSystem.getInstance();
        DriveTrainSystem.getInstance().initHardware();
        OI.getInstance().getXboxController();
        /*motor_rightf = new WPI_TalonSRX(13);
        motor_rightb = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor_leftf = new WPI_TalonSRX(20);
        motor_leftb = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);

        motor_leftf.setInverted(true);
        motor_leftb.setInverted(true);

        motor_rightf.configFactoryDefault();
        motor_rightb.restoreFactoryDefaults();
        motor_leftf.configFactoryDefault();
        motor_leftb.restoreFactoryDefaults();
        //PID.initHardware();*/
        //TODO code outake system
    }

    @Override
    public void robotPeriodic() {
        DriveTrainSystem.getInstance().run();
        DriveTrainSystem.getInstance().executePID();

    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        //OI.getInstance().controls();

    }

    @Override
    public void teleopPeriodic() {
        //call drive train system


        //TODO create OI function

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {
    }
}
