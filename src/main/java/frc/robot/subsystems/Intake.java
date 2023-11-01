package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

    private CANSparkMax motor;
    private DigitalInput limitSwitch;

    private boolean zeroed = false;

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
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() && getLimitSwitchTripped() && !zeroed) {
            zeroIntake();
        }
    }

    public void setMotor(double speed) {
        motor.set(speed);
    }

    public void setPosition(double radians) {
        motor.getPIDController().setReference(radians, CANSparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public double getPositionError(double target) {
        return Math.abs(motor.getEncoder().getPosition() - target);
    }

    public boolean getLimitSwitchTripped() {
        return !limitSwitch.get();
    }

    public void setEncoderPosition(double pos) {
        motor.getEncoder().setPosition(pos);
    }

    public void zeroIntake() {
        setEncoderPosition(Constants.IntakeConstants.ZERO_POSITION);
        zeroed = true;
    }

    public boolean hasBeenZeroed() {
        return zeroed;
    }
}
