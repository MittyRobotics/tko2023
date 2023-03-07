package com.github.mittyrobotics.telescope;

import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotConstants;
import com.github.mittyrobotics.telescope.commands.ExtensionToKinematics;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import com.github.mittyrobotics.util.interfaces.ISubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.github.mittyrobotics.intake.StateMachine.ProfileState.*;

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
        telescopeNeo.restoreFactoryDefaults();
        telescopeNeo.getPIDController().setP(TelescopeConstants.DEFAULT_P);
        telescopeNeo.getPIDController().setI(TelescopeConstants.DEFAULT_I);
        telescopeNeo.getPIDController().setD(TelescopeConstants.DEFAULT_D);
        telescopeNeo.getPIDController().setFF(0.1/450 * 0.8);
        telescopeNeo.getPIDController().setOutputRange(-0.5, 0.5);
        telescopeNeo.getEncoder().setPosition(0);
        telescopeNeo.getPIDController().setFeedbackDevice(telescopeNeo.getEncoder());
        telescopeNeo.setIdleMode(CANSparkMax.IdleMode.kBrake);
//        telescopeNeo.getPIDController().setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
//        telescopeNeo.getPIDController().setSmartMotionMaxAccel(0.001, 0);
//        telescopeNeo.getPIDController().setSmartMotionMaxVelocity(0.01, 0);
//        telescopeNeo.getPIDController().setSmartMotionMinOutputVelocity(0.05, 0);
//        telescopeNeo.getPIDController().setSmartMotionAllowedClosedLoopError(0, 0);
//        telescopeNeo.setClosedLoopRampRate(1.5);



//        telescopeNeo.getPIDController().setSmartMotionMaxAccel(5/100. / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 0);
//        telescopeNeo.getPIDController().setSmartMotionMaxVelocity(10/100. / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 0);
//        telescopeNeo.getPIDController().setSmartMotionMaxAccel(5 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 0);
//        telescopeNeo.getPIDController().setSmartMotionMaxVelocity(10 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * 60, 0);
//        telescopeNeo.getPIDController().setSmartMotionMinOutputVelocity(TelescopeConstants.MIN_VELOCITY, 0);
//        telescopeNeo.getPIDController().setOutputRange(-0.2, 0.2);

        halifaxMax = new DigitalInput(TelescopeConstants.HALIFAX_MAX_CHANNEL);
        halifaxMin = new DigitalInput(TelescopeConstants.HALIFAX_MIN_CHANNEL);

        createMPs();

        setDefaultCommand(new ExtensionToKinematics());
        System.out.println("AFTER SET DEFAULT");
    }

    private void createMPs() {
        TelescopeConstants.TELESCOPE_MPS.put(DEFAULT,
                new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2));
        TelescopeConstants.TELESCOPE_MPS.put(STOWED_TO_MID,
                new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2));
        TelescopeConstants.TELESCOPE_MPS.put(STOWED_TO_HIGH,
                new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2));
        TelescopeConstants.TELESCOPE_MPS.put(STOWED_TO_HP,
                new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2));
        TelescopeConstants.TELESCOPE_MPS.put(GROUND_TO_STOWED,
                new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2));
        TelescopeConstants.TELESCOPE_MPS.put(MID_TO_STOWED,
                new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2));
        TelescopeConstants.TELESCOPE_MPS.put(HIGH_TO_STOWED,
                new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2));
        TelescopeConstants.TELESCOPE_MPS.put(HP_TO_STOWED,
                new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2));
        TelescopeConstants.TELESCOPE_MPS.put(MID_TO_HIGH,
                new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2));
        TelescopeConstants.TELESCOPE_MPS.put(HIGH_TO_MID,
                new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2));
    }

    public void setMotor(double percentOutput) {
        telescopeNeo.set(percentOutput);
    }

    public CANSparkMax getNeo() {
        return telescopeNeo;
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

    public void setFF(double FF) {
        telescopeNeo.getPIDController().setFF(FF);
    }

    public void setPositionMeters(double meters) {
        telescopeNeo.getPIDController().setReference(meters/TelescopeConstants.METERS_PER_MOTOR_REV, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setPositionInches(double inches) {
        telescopeNeo.getPIDController().setReference((inches/39.37)/TelescopeConstants.METERS_PER_MOTOR_REV, CANSparkMax.ControlType.kPosition);
    }

    public void setVelocityMetersPerSecond(double meters) {
        telescopeNeo.getPIDController().setReference((meters/TelescopeConstants.METERS_PER_MOTOR_REV) / 10, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelocityInches(double inches) {
        setVelocityMetersPerSecond(inches / 39.37);
    }

    public double getOutput() {
        return telescopeNeo.getAppliedOutput();
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

    public double rawPos() {
        return telescopeNeo.getEncoder().getPosition();
    }

    public double rawVel() {
        return telescopeNeo.getEncoder().getVelocity();
    }

    public void setRaw(double vel) {
        telescopeNeo.getPIDController().setReference(vel, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void periodic() {
//        if (getVelocityInchesPerSecond() > 2) {
//            setVelocityInches(2);
//        }
    }
}
