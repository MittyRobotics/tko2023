package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ConveyorConstants.*;

public class Conveyor extends SubsystemBase {

    private CANSparkMax motor;
    private DigitalInput limitSwitch;

    public Conveyor() {
        initHardware();
    }

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
}
