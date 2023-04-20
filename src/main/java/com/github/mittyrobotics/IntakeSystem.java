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
        OI.getInstance().controls();
    }

    public void setIntakeDown() {
        intakeMotor.set(ControlMode.Position, 45);
    }

    public void setIntakeUp() {
        intakeMotor.set(ControlMode.Position, -45);
    }

    public void stop()
    {
        intakeMotor.set(0);
    }


    public void run()
    {
        OI.getInstance().controls();
    }


}





