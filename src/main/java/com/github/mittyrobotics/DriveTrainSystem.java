package com.github.mittyrobotics;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class DriveTrainSystem extends SubsystemBase {
    private static DriveTrainSystem ourInstance = new DriveTrainSystem();
    public static DriveTrainSystem getInstance() {return ourInstance;}
    private DriveTrainSystem() {
        super();
    }
    WPI_TalonSRX motor_rightf, motor_leftf;
    CANSparkMax motor_rightb, motor_leftb;
    private PIDController pid;
    private Encoder encoderlb;
    private Encoder encoderrb;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    public void initHardware()
    {
        //change ids! otherwise only motor_leftb will run, everything else is overridden - naomi
        motor_leftf = new WPI_TalonSRX(13);
        motor_rightb = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor_rightf = new WPI_TalonSRX(22);
        motor_leftb = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);

        kP = kD = kI = 0;
        pid = new PIDController(kP, kI, kD);
        encoderlb = new Encoder(0, 1);
        encoderrb = new Encoder(0, 1);
        pid.setTolerance(5, 10);
        pid.setSetpoint(1);


        motor_rightf.configFactoryDefault();
        motor_rightb.restoreFactoryDefaults();
        motor_leftf.configFactoryDefault();
        motor_leftb.restoreFactoryDefaults();
        motor_leftf.setInverted(true);
        motor_leftb.setInverted(true);
    }

    @Override
    public void periodic()
    {
        //turns all motors off, why? - naomi
        /*motor_rightf.set(ControlMode.PercentOutput, OI.);
        motor_rightb.set(ControlMode.PercentOutput, 0);
        motor_leftf.set(ControlMode.PercentOutput, 0);
        motor_leftb.set(ControlMode.PercentOutput, 0);*/
        run();
    }


    public void run(){
        double right, left;
        right = OI.getInstance().rjoystick();
        left = OI.getInstance().ljoystick();

        double rpower = 0, lpower = 0;
        if(right<-0.2)
        {
            lpower += right;
            rpower -= right;
        }
        else if(right>0.2)
        {
            rpower += right;
            lpower -= right;
        }
        if(left> 0.2) {rpower += left; lpower+= left;}
        else if(left<-0.2) {rpower = left; lpower = left;}



        motor_rightf.set(rpower);
        motor_rightb.set(rpower);
        motor_leftf.set(lpower);
        motor_leftb.set(lpower);
        //figure out how to convert joystick to motor power
    }

    public void executePID() {
        motor_leftf.set(pid.calculate(encoderlb.getDistance(), pid.getSetpoint()));
        motor_leftb.set(pid.calculate(encoderlb.getDistance(), pid.getSetpoint()));
        motor_rightf.set(pid.calculate(encoderrb.getDistance(), pid.getSetpoint()));
        motor_rightb.set(pid.calculate(encoderrb.getDistance(), pid.getSetpoint()));
        if(pid.atSetpoint())
        {
            motor_leftf.set(0);
            motor_leftb.set(0);
            motor_rightf.set(0);
            motor_rightb.set(0);
        }
    }
}

/*rpower = left + right;
        lpower = left - right;"*/