package com.github.mittyrobotics.armclaw;

import com.github.mittyrobotics.util.interfaces.IMotorSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawGrabberSubsystem extends SubsystemBase implements IMotorSubsystem {
    public static CANSparkMax grabberSpark;
    private static ClawGrabberSubsystem instance;
    private double initialPosition = 0;
    private static DigitalInput clawProxSensor;

    private ClawGrabberSubsystem(){
        super();
        setName("ClawGrabber");
    }
    public static ClawGrabberSubsystem getInstance(){
        if(instance==null){
            instance = new ClawGrabberSubsystem();
        }
        return instance;
    }

    @Override
    public void updateDashboard() {

    }

    @Override
    public void initHardware() {
        grabberSpark = new CANSparkMax(ClawConstants.GRABBER_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        grabberSpark.restoreFactoryDefaults();
        grabberSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        grabberSpark.setInverted(ClawConstants.GRABBER_SPARK_INVERTED);
        clawProxSensor = new DigitalInput(ClawConstants.CLAW_PROX_SENSOR_CHANNEL);
    }

    @Override
    public void overrideSetMotor(double percent) {

    }

    public void setGrabberAngle(double angle){
        grabberSpark.getPIDController().setReference(angle*ClawConstants.GRABBER_ROTATIONS_PER_DEGREE, CANSparkMax.ControlType.kPosition);
    }

    public boolean pieceHeld(){
        return !clawProxSensor.get();
    }
}
