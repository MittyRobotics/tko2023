package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {

    private CANSparkMax[] motor;
    private boolean isShooting = false;
    private double targetRPM = 0;

    public Shooter() {
        initHardware();
    }

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
        targetRPM = rpm;
        motor[0].getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
        motor[1].getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    public double getVelocityError() {
        return Math.abs(targetRPM - motor[0].getEncoder().getVelocity());
    }
}
