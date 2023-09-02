package com.github.mittyrobotics.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import static com.github.mittyrobotics.intake.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance() {
        if (instance == null) instance = new IntakeSubsystem();
        return instance;
    }

    private CANSparkMax motor;
    private ArrayList<Double> current = new ArrayList<>();

    public void initHardware() {
        motor = new CANSparkMax(SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(INVERTED);
    }

    public void setMotor(double percent) {
        motor.set(percent);
    }

    public void updateCurrent() {
        current.add(motor.getOutputCurrent());
        if (current.size() > 10) current.remove(0);
    }

    public double averagedCurrent() {
        double sum = 0;
        for (double c : current) {
            sum += c;
        }
        return sum / 10;
    }

    public boolean intakeFull() {
        return averagedCurrent() > 29.5;
    }
}
