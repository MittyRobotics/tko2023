package com.github.mittyrobotics;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.button.Button;


public class IntakeSystem extends SubsystemBase {
    private static IntakeSystem ourInstance = new IntakeSystem();
    public static IntakeSystem getInstance() {return ourInstance;}
    private IntakeSystem() {
        super();
    }
    WPI_TalonSRX intakeMotor;
    public void initHardware()
    {
        //update ids - naomi
        intakeMotor = new WPI_TalonSRX(4);
        intakeMotor.configFactoryDefault();
        //intakeMotor.setInverted(true);
    }


    @Override
    public void periodic()
    {
        double right;
        right = OI.getInstance().controls_intake();
        if(right<-0.2) {
            intakeMotor.set(-1*right);
        }

    }


}





