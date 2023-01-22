package com.github.mittyrobotics.intake;

import com.github.mittyrobotics.util.interfaces.IMotorSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawGrabberSubsystem extends SubsystemBase implements IMotorSubsystem {
    public static CANSparkMax grabberSpark;
    private static ClawGrabberSubsystem instance;
    private double initialPosition = 0;
    private static DigitalInput clawProxSensor;
    private static boolean isCone = false;

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
        grabberSpark = new CANSparkMax(IntakeConstants.GRABBER_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        grabberSpark.restoreFactoryDefaults();
        grabberSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        grabberSpark.setInverted(IntakeConstants.GRABBER_SPARK_INVERTED);
        grabberSpark.getPIDController().setFeedbackDevice(grabberSpark.getEncoder());

        clawProxSensor = new DigitalInput(IntakeConstants.CLAW_PROX_SENSOR_CHANNEL);
    }

    @Override
    public void overrideSetMotor(double percent) {


    }
    /* sets grabber claws at specific angle, 0 = fully closed */
    public void setGrabberAngle(double angle){
        grabberSpark.getPIDController().setReference(angle*IntakeConstants.GRABBER_ROTATIONS_PER_DEGREE, CANSparkMax.ControlType.kPosition);
    }


    /* returns true if prox sensor detects a game piece within range and the claws are in contact with the game piece */
    public boolean pieceHeld(){
        double lowerThreshold;
        double upperThreshold;
        double currentPosition = grabberSpark.getEncoder().getPosition()*180;

        if(isCone){
            //TODO: tune upper/lower threshold values
            lowerThreshold=IntakeConstants.CONE_DEGREES-5;
            upperThreshold=IntakeConstants.CONE_DEGREES+5;
        }
        else{
            lowerThreshold=IntakeConstants.CUBE_DEGREES-5;
            upperThreshold=IntakeConstants.CUBE_DEGREES+5;
        }

        return !clawProxSensor.get() && (lowerThreshold < currentPosition && upperThreshold > currentPosition);
    }

    /*  sets the current state of the ClawGrabberSubsystem to cone or cube, called in grab command, used above ^ to specify the thresholds
     */

    public void setConeOrCube(boolean isCone){
        if(isCone){
            this.isCone = true;
        }
        else{
            this.isCone = false;
        }
    }
}
