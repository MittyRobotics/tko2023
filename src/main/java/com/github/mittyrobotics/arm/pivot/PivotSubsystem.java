package com.github.mittyrobotics.arm.pivot;

import com.github.mittyrobotics.util.math.Angle;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.github.mittyrobotics.arm.pivot.PivotConstants.*;

public class PivotSubsystem extends SubsystemBase {
    private static PivotSubsystem instance;

    public static PivotSubsystem getInstance() {
        if (instance == null) instance = new PivotSubsystem();
        return instance;
    }

    private CANSparkMax[] motor;

    public void initHardware() {
        for (int i = 0; i < 2; i++) {
            motor[i] = new CANSparkMax(SPARK_ID[i], CANSparkMaxLowLevel.MotorType.kBrushless);
            motor[i].restoreFactoryDefaults();
            motor[i].setInverted(INVERTED[i]);
            motor[i].getPIDController().setP(PID[0]);
            motor[i].getPIDController().setI(PID[1]);
            motor[i].getPIDController().setD(PID[2]);
            motor[i].getEncoder().setPositionConversionFactor(RADIANS_PER_REV);
            motor[i].getEncoder().setPosition(0);
        }
    }

    public void setRaw(double rpm, double ff) {
        motor[0].getPIDController().setReference(
                rpm, CANSparkMax.ControlType.kVelocity, 0, ff, SparkMaxPIDController.ArbFFUnits.kPercentOut);
        motor[1].getPIDController().setReference(
                rpm, CANSparkMax.ControlType.kVelocity, 0, ff, SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }

    public Angle getCurrentAngle() {
        return new Angle(motor[0].getEncoder().getPosition(), true);
    }
}
