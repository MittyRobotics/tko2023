/*package com.github.mittyrobotics;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class OutakeSystem extends SubsystemBase{


    private static OutakeSystem ourInstance = new OutakeSystem();
    public static OutakeSystem getInstance() {return ourInstance;}
    private OutakeSystem() {
        super();
    }
    WPI_TalonSRX rampMotorR, rampMotorL, intakeMotor;
    public void initHardware()
    {
        rampMotorR = new WPI_TalonSRX(24);
        rampMotorL = new WPI_TalonSRX(24);
        intakeMotor.configFactoryDefault();
        rampMotorR.configFactoryDefault();
        rampMotorL.configFactoryDefault();
        rampMotorR.setInverted(true);
    }


    @Override
    public void periodic()
    {
        OI.getInstance().controls();
    }

    public void stop()
    {
        intakeMotor.set(0);
        rampMotorR.set(0);
        rampMotorL.set(0);

    }


    public void run()
    {
        OI.getInstance().controls();
    }
}
*/