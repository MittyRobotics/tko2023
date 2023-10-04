package com.github.mittyrobotics.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.github.mittyrobotics.shooter.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;

    public static ShooterSubsystem getInstance() {
        if (instance == null) instance = new ShooterSubsystem();

        return instance;
    }

    private CANSparkMax[] motor;
    private boolean isShooting = false;

    public void initHardware() {
        motor = new CANSparkMax[2];
        for (int i = 0; i < 2; i++) {
            motor[i] = new CANSparkMax(MOTOR_ID[i], CANSparkMaxLowLevel.MotorType.kBrushless);
            motor[i].restoreFactoryDefaults();
            motor[i].getEncoder().setPositionConversionFactor(1. / MOTOR_TURNS_PER_REV);
            motor[i].getEncoder().setPositionConversionFactor(1. / MOTOR_TURNS_PER_REV);
            motor[i].getEncoder().setPosition(0);
            motor[i].getPIDController().setP(PID[0]);
            motor[i].getPIDController().setI(PID[1]);
            motor[i].getPIDController().setD(PID[2]);
            motor[i].getPIDController().setFF(PID[3]);
        }
    }

    public void setMotor(double speed) {
        motor[0].set(speed);
        motor[1].set(speed);
    }

    public void setSpeed(double rpm) {
        motor[0].getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
        motor[1].getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    public boolean isShooting() {
        return isShooting;
    }

    public void stopShooting() {
        isShooting = false;
    }

    public void startShooting() {
        isShooting = true;
    }
}
