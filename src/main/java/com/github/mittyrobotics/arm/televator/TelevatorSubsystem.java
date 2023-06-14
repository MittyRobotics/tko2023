package com.github.mittyrobotics.arm.televator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import static com.github.mittyrobotics.arm.televator.TelevatorConstants.*;

public class TelevatorSubsystem {
    private static TelevatorSubsystem instance;

    public static TelevatorSubsystem getInstance() {
        if (instance == null) instance = new TelevatorSubsystem();
        return instance;
    }

    private CANSparkMax motor;

    public void initHardware() {
        motor = new CANSparkMax(SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(INVERTED);
        motor.getPIDController().setP(PID[0]);
        motor.getPIDController().setI(PID[1]);
        motor.getPIDController().setD(PID[2]);
        motor.getPIDController().setFF(PID[3]);
        motor.getEncoder().setPosition(0);
    }

    public void setRaw() {
        
    }
}
