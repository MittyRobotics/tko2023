package com.github.mittyrobotics;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class OutakeSystem extends SubsystemBase{


    private static OutakeSystem ourInstance = new OutakeSystem();
    public static OutakeSystem getInstance() {return ourInstance;}
    private OutakeSystem() {
        super();
    }
    WPI_TalonSRX rampMotorR, rampMotorL;
    public void initHardware()
    {
        rampMotorR = new WPI_TalonSRX(13);
        rampMotorL = new WPI_TalonSRX(20);
        rampMotorR.configFactoryDefault();
        rampMotorL.configFactoryDefault();
    }


    @Override
    public void periodic() {
        double right;
        right = OI.getInstance().controls_outake();
        if(right>0.2)
        {
            rampMotorR.set(right);
            rampMotorL.set(right);
        }



    }

    public void runauto(){
        rampMotorR.set(50);
        rampMotorL.set(50);
    }
}
