package com.github.mittyrobotics;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class Robot extends TimedRobot {

    WPI_TalonSRX motor_rightf, motor_leftf;
    CANSparkMax motor_rightb, motor_leftb;

    DoubleArraySubscriber areasSub;
    NetworkTable table;

    @Override
    public void robotInit() {
        //IntakeSystem.getInstance();
        //IntakeSystem.getInstance().initHardware();
        //OutakeSystem.getInstance();
        //OutakeSystem.getInstance().initHardware();
        //OI.getInstance();
        //limelight.getInstance().initHardware();
        //DriveTrainSystem.getInstance().initHardware();
        table = NetworkTableInstance.getDefault().getTable("limelight");
        //areasSub = table.getDoubleTopic("tx").subscribe(new double {});


        //OI.getInstance().getXboxController();

        //OI.getInstance().getXboxController2();
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
        //CommandScheduler.getInstance().run();
        //OutakeSystem.getInstance().periodic();
        //IntakeSystem.getInstance().periodic();


    }

    @Override
    public void autonomousInit() {
        //CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new DriveTrainCommand()));
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void autonomousPeriodic() {
        //DriveTrainSystem.getInstance().encoder_value();
        //DriveTrainSystem.getInstance().executePID();
        //OutakeSystem.getInstance().runauto();
    }

    @Override
    public void teleopInit() {
        //OI.getInstance().controls();

    }

    @Override
    public void teleopPeriodic() {
        //call drive train system
        //DriveTrainSystem.getInstance().executePID();
        //DriveTrainSystem.getInstance().aa();
        //DriveTrainSystem.getInstance().trapezoid();
        //System.out.println(OI.getInstance().buttonpress());
        //limelight.getInstance().periodic();

        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        boolean target = tv.getBoolean(false);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        //SmartDashboard.putBoolean("LimelightTarget", target);

        



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
