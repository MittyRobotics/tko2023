package com.github.mittyrobotics.arm.televator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.github.mittyrobotics.arm.televator.TelevatorConstants.*;

public class TelevatorSubsystem extends SubsystemBase {
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
        motor.getEncoder().setPositionConversionFactor(INCHES_PER_REV);
        motor.getEncoder().setPosition(0);
    }

    public void setRaw(double rpm, double ff) {
        motor.getPIDController().setReference(
                rpm, CANSparkMax.ControlType.kVelocity, 0, ff, SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }

    public double getCurrentExtension() {
        return motor.getEncoder().getPosition();
    }
}
