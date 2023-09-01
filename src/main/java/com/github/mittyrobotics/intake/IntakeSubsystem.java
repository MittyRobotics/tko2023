package com.github.mittyrobotics.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.github.mittyrobotics.intake.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance() {
        if (instance == null) instance = new IntakeSubsystem();
        return instance;
    }

    private CANSparkMax motor;

    public void initHardware() {
        motor = new CANSparkMax(SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(INVERTED);
    }

    public void setMotor(double percent) {
        motor.set(percent);
    }

    public boolean intakeFull() {
        return motor.getOutputCurrent() > F;
    }
}
