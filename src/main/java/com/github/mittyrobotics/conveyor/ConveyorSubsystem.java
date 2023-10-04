package com.github.mittyrobotics.conveyor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.github.mittyrobotics.conveyor.ConveyorConstants.*;

public class ConveyorSubsystem extends SubsystemBase {
    private static ConveyorSubsystem instance;

    public static ConveyorSubsystem getInstance() {
        if (instance == null) instance = new ConveyorSubsystem();

        return instance;
    }

    private ConveyorSubsystem() {

    }

    private CANSparkMax motor;
    private DigitalInput limitSwitch;
    private boolean isIntaking = false;

    public void initHardware() {
        motor = new CANSparkMax(MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.getEncoder().setPositionConversionFactor(INCHES_PER_REV);
        motor.getEncoder().setVelocityConversionFactor(INCHES_PER_REV / 60);

        limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);
    }

    public void setMotor(double speed) {
        motor.set(speed);
    }

    public boolean getLimitSwitchTripped() {
        return !limitSwitch.get();
    }

    public boolean isIntaking() {
        return isIntaking;
    }

    public void stopIntaking() {
        isIntaking = false;
    }

    public void startIntaking() {
        isIntaking = true;
    }
}
