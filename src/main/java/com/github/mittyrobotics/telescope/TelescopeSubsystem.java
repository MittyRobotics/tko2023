package com.github.mittyrobotics.telescope;

import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.telescope.commands.ExtensionToKinematics;
import com.github.mittyrobotics.util.interfaces.ISubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopeSubsystem extends SubsystemBase implements ISubsystem {
    CANSparkMax telescopeNeo;

    DigitalInput halifaxMax;
    DigitalInput halifaxMin;

    private static TelescopeSubsystem instance;

    public static TelescopeSubsystem getInstance() {
        if(instance == null) {
            instance = new TelescopeSubsystem();
        }
        return instance;
    }

    public TelescopeSubsystem() {
        super();
        setName("Telescope");
    }

    @Override
    public void updateDashboard() {

    }

    @Override
    public void initHardware() {
        telescopeNeo = new CANSparkMax(TelescopeConstants.TELESCOPE_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        telescopeNeo.getPIDController().setP(TelescopeConstants.DEFAULT_P);
        telescopeNeo.getPIDController().setI(TelescopeConstants.DEFAULT_I);
        telescopeNeo.getPIDController().setD(TelescopeConstants.DEFAULT_D);
        telescopeNeo.restoreFactoryDefaults();
        telescopeNeo.getEncoder().setPosition(0);
        telescopeNeo.getPIDController().setFeedbackDevice(telescopeNeo.getEncoder());
        telescopeNeo.setIdleMode(CANSparkMax.IdleMode.kBrake);
        telescopeNeo.getPIDController().setSmartMotionMaxAccel(5/100. / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 0);
        telescopeNeo.getPIDController().setSmartMotionMaxVelocity(10/100. / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 0);
//        telescopeNeo.getPIDController().setSmartMotionMaxAccel(5 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 0);
//        telescopeNeo.getPIDController().setSmartMotionMaxVelocity(10 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 0);
//        telescopeNeo.getPIDController().setSmartMotionMinOutputVelocity(TelescopeConstants.MIN_VELOCITY, 0);

        halifaxMax = new DigitalInput(TelescopeConstants.HALIFAX_MAX_CHANNEL);
        halifaxMin = new DigitalInput(TelescopeConstants.HALIFAX_MIN_CHANNEL);

        setDefaultCommand(new ExtensionToKinematics());
        System.out.println("AFTER SET DEFAULT");
    }

    public void setMotor(double percentOutput) {
        telescopeNeo.set(percentOutput);
    }

    public void setBrakeMode() {
            telescopeNeo.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoastMode() {
            telescopeNeo.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setPID(double P, double I, double D) {
        telescopeNeo.getPIDController().setP(P);
        telescopeNeo.getPIDController().setI(I);
        telescopeNeo.getPIDController().setD(D);
    }

    public void setPositionMeters(double meters) {
        telescopeNeo.getPIDController().setReference(meters/TelescopeConstants.METERS_PER_MOTOR_REV, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setPositionInches(double inches) {
        telescopeNeo.getPIDController().setReference((inches/39.37)/TelescopeConstants.METERS_PER_MOTOR_REV, CANSparkMax.ControlType.kSmartMotion);
    }

    public double getDistanceMeters() {
        return telescopeNeo.getEncoder().getPosition() * TelescopeConstants.METERS_PER_MOTOR_REV;
    }

    public double getDistanceInches() {
        return getDistanceMeters() * 39.37;
    }

    public double getVelocityMetersPerSecond() {
        return (telescopeNeo.getEncoder().getVelocity() / 60.) * TelescopeConstants.METERS_PER_MOTOR_REV;
    }

    public double getVelocityInchesPerSecond() {
        return ((telescopeNeo.getEncoder().getVelocity() / 60.) * TelescopeConstants.METERS_PER_MOTOR_REV) * 39.37;
    }

    public boolean withinThreshold() {
        return Math.abs(ArmKinematics.getTelescopeDesired() - getDistanceMeters()) < TelescopeConstants.EXTENSION_THRESHOLD;
    }

    public boolean getHalifaxMaxContact() {
        return !halifaxMax.get();
    }

    public boolean getHalifaxMinContact() {
        return !halifaxMin.get();
    }

    public void resetMeters(double meters) {
        telescopeNeo.getEncoder().setPosition(meters / TelescopeConstants.METERS_PER_MOTOR_REV);
    }

    public void resetInches(double inches) {
        telescopeNeo.getEncoder().setPosition((inches / 39.37) / TelescopeConstants.METERS_PER_MOTOR_REV);
    }

    public void setVelZero() {
        telescopeNeo.set(0);
    }

}
