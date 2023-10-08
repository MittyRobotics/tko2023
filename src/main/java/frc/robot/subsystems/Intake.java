package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

    private CANSparkMax motor;

    public Intake() {
        initHardware();
    }

    public void initHardware() {
        motor = new CANSparkMax(MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.getEncoder().setPositionConversionFactor(RADIANS_PER_REV);
        motor.getEncoder().setVelocityConversionFactor(RADIANS_PER_REV / 60);
        motor.getEncoder().setPosition(0);
        motor.getPIDController().setP(PID[0]);
        motor.getPIDController().setI(PID[1]);
        motor.getPIDController().setD(PID[2]);
        motor.getPIDController().setFF(PID[3]);
    }

    public void setMotor(double speed) {
        motor.set(speed);
    }

    public void setPosition(double radians) {
        motor.getPIDController().setReference(radians, CANSparkMax.ControlType.kPosition);
    }

    public double getPositionError(double target) {
        return Math.abs(motor.getEncoder().getPosition() - target);
    }
}
