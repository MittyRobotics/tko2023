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
        intakeMotor = new WPI_TalonSRX(24);
        intakeMotor.configFactoryDefault();
    }


    @Override
    public void periodic()
    {
        while(OI.getInstance().controls_intake() == true)
        {
            intakeMotor.set(50);
        }
        intakeMotor.set(0);

    }


}





