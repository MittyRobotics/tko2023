package com.github.mittyrobotics.telescope;

import com.github.mittyrobotics.util.interfaces.ISubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopeSubsystem extends SubsystemBase implements ISubsystem {
    CANSparkMax telescopeNeo;


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
        telescopeNeo.getPIDController().setFeedbackDevice(telescopeNeo.getEncoder());
        telescopeNeo.setIdleMode(CANSparkMax.IdleMode.kBrake);
        telescopeNeo.getPIDController().setSmartMotionMaxVelocity(TelescopeConstants.MAX_VELOCITY, 0);
        telescopeNeo.getPIDController().setSmartMotionMinOutputVelocity(TelescopeConstants.MIN_VELOCITY, 0);
        telescopeNeo.getPIDController().setSmartMotionMaxAccel(TelescopeConstants.MAX_ACCEL, 0);
    }

    public void setPID(double P, double I, double D) {
        telescopeNeo.getPIDController().setP(P);
        telescopeNeo.getPIDController().setI(I);
        telescopeNeo.getPIDController().setD(D);
    }

    public void setPositionMeters(double meters) {
        telescopeNeo.getPIDController().setReference(meters/TelescopeConstants.METERS_PER_REV, CANSparkMax.ControlType.kSmartMotion);

    }

    public void setPositionInches(double inches) {
        telescopeNeo.getPIDController().setReference((inches/39.37)/TelescopeConstants.METERS_PER_REV, CANSparkMax.ControlType.kPosition);
    }

    public double getDistanceMeters() {
        return telescopeNeo.getEncoder().getPosition() * TelescopeConstants.METERS_PER_REV;
    }

    public double getDistanceInches() {
        return getDistanceMeters() * 39.37;
    }

    public double getVelocityMetersPerSecond() {
        return (telescopeNeo.getEncoder().getVelocity() / 60.) * TelescopeConstants.METERS_PER_REV;
    }

    public double getVelocityInchesPerSecond() {
        return ((telescopeNeo.getEncoder().getVelocity() / 60.) * TelescopeConstants.METERS_PER_REV) * 39.37;
    }
}
