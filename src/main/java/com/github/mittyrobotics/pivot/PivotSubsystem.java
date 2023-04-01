package com.github.mittyrobotics.pivot;

import com.github.mittyrobotics.pivot.commands.PivotToKinematics;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.github.mittyrobotics.intake.StateMachine.ProfileState.*;

public class PivotSubsystem extends SubsystemBase {
    private static PivotSubsystem instance;
    private CANSparkMax[] spark = new CANSparkMax[2];
    private DigitalInput halifaxTop;
    private DigitalInput halifaxBottom;

    public static PivotSubsystem getInstance() {
        instance = instance == null ? new PivotSubsystem() : instance;
        return instance;
    }

    private PivotSubsystem() {

    }

    public void initHardware() {
        for (int i = 0; i < 2; i++) {
            spark[i] = new CANSparkMax(PivotConstants.PIVOT_ID[i], CANSparkMax.MotorType.kBrushless);
            spark[i].restoreFactoryDefaults();
            spark[i].getEncoder().setPosition(0);
            spark[i].setIdleMode(CANSparkMax.IdleMode.kBrake);
            spark[i].getPIDController().setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
            spark[i].getPIDController().setSmartMotionMaxAccel(30. / 360 * 60, 0);
            spark[i].getPIDController().setSmartMotionMaxVelocity(40. / 360 * 60, 0);
            spark[i].getPIDController().setFeedbackDevice(spark[i].getEncoder());
//            spark[i].getPIDController().setFF();
//            spark[i].getPIDController().setOutputRange(-0.1, 0.1);
//            spark[i].setClosedLoopRampRate(1.5);
//            spark[i].getPIDController().setSmartMotionAllowedClosedLoopError(1, 0);
        }

        halifaxTop = new DigitalInput(PivotConstants.HALIFAX_TOP_CHANNEL);
        halifaxBottom = new DigitalInput(PivotConstants.HALIFAX_BOTTOM_CHANNEL);

        createMPs();

        System.out.println(PivotSubsystem.getInstance().getPositionRadians());

        setDefaultCommand(new PivotToKinematics());
    }

    private void createMPs() {
        PivotConstants.PIVOT_MPS.put(DEFAULT,
                new TrapezoidalMotionProfile(1000 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 210 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 720 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(STOWED_TO_GROUND,
                new TrapezoidalMotionProfile(1600 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 500 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 1200 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(STOWED_TO_MID,
                new TrapezoidalMotionProfile(1200 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 250 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 860 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(STOWED_TO_HIGH,
                new TrapezoidalMotionProfile(1200 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 250 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 860 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(STOWED_TO_HP,
                new TrapezoidalMotionProfile(1000 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 250 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 720 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(GROUND_TO_STOWED,
                new TrapezoidalMotionProfile(800 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 210 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 720 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(MID_TO_STOWED,
                new TrapezoidalMotionProfile(400 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 90 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 360 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(HIGH_TO_STOWED,
                new TrapezoidalMotionProfile(400 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 90 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 360 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(HP_TO_STOWED,
                new TrapezoidalMotionProfile(800 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 210 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 720 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(MID_TO_HIGH,
                new TrapezoidalMotionProfile(400 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 90 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 720 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(HIGH_TO_MID,
                new TrapezoidalMotionProfile(400 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 90 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 720 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
        PivotConstants.PIVOT_MPS.put(SCORING_TO_STOWED,
                new TrapezoidalMotionProfile(400 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 150 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 360 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, 0 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 30 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3));
    }

    public void setBrakeMode() {
        for (int i = 0; i < 2; i++) {
            spark[i].setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
    }

    public void setCoastMode() {
        for (int i = 0; i < 2; i++) {
            spark[i].setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
    }

    public void setPositionRadians(double radians) {
        spark[0].getPIDController().setReference(radians / (2 * Math.PI) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, CANSparkMax.ControlType.kSmartMotion);
        spark[1].getPIDController().setReference(radians / (2 * Math.PI) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setPositionDegrees(double degrees) {
        setPositionRadians(Math.PI / 180 * degrees);
    }

    //check unit conversions if u going to use set vel
    public void setVelocityRadiansPerSecond(double radiansPerSecond) {
        System.out.println((radiansPerSecond / (2 * Math.PI) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO) * 60);
       spark[0].getPIDController().setReference((radiansPerSecond / (2 * Math.PI) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO) * 60, CANSparkMax.ControlType.kVelocity);
       spark[1].getPIDController().setReference((radiansPerSecond / (2 * Math.PI) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO) * 60, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelocityDegreesPerSecond(double degreesPerSecond) {
        setVelocityRadiansPerSecond(degreesPerSecond * Math.PI / 180);
    }
// 25 in/s, 90 deg/s
    public double getPositionRadians() {
        return (spark[0].getEncoder().getPosition() * 2 * Math.PI) * PivotConstants.PIVOT_TO_NEO_GEAR_RATIO;
    }

    public double getPositionDegrees() {
        return getPositionRadians() * 180 / (Math.PI);
    }

    public double getVelocityRadiansPerSecond() {
        return ((spark[0].getEncoder().getVelocity() * 2 * Math.PI / 60) * PivotConstants.PIVOT_TO_NEO_GEAR_RATIO);
    }

    public double getVelocityDegreesPerSecond() {
        return getVelocityRadiansPerSecond() * 180 / (2 * Math.PI);
    }

    public void configPID (double p, double i, double d) {
        for (int j = 0; j < 2; j++) {
            spark[j].getPIDController().setP(p);
            spark[j].getPIDController().setI(i);
            spark[j].getPIDController().setD(d);
        }
    }

    public void setFF(double val) {
        spark[0].getPIDController().setFF(val);
        spark[1].getPIDController().setFF(val);
    }

    public boolean withinThreshold() {
        return Math.abs(ArmKinematics.getPivotDesired().getRadians() - getPositionRadians()) < PivotConstants.PIVOT_THRESHOLD;
    }

    public boolean withinDeadzoneThreshold() {
        return Math.abs(ArmKinematics.getPivotDesired().getRadians() - getPositionRadians()) < PivotConstants.PIVOT_DEADZONE_THRESHOLD;
    }

    public void resetAngleDegrees(double degrees) {
        spark[0].getEncoder().setPosition((degrees / 360) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO);
        spark[1].getEncoder().setPosition((degrees / 360) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO);
    }

    public void resetAngleRadians(double radians) {
        spark[0].getEncoder().setPosition((radians / (2* Math.PI)) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO);
        spark[1].getEncoder().setPosition((radians / (2* Math.PI)) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO);
    }

    public boolean getHalifaxTopContact() {
        return !halifaxTop.get();
    }

    public boolean getHalifaxBottomContact() {
        return !halifaxBottom.get();
    }

    public void setVelZero() {
        for (int i = 0; i < 2; i++) {
            spark[i].set(0);
        }
    }

    public void setMotor(double val) {
        spark[0].set(val);
        spark[1].set(val);
    }

    public double rawVel() {
        return spark[0].getEncoder().getVelocity();
    }

    public double rawPos() {
        return spark[0].getEncoder().getPosition();
    }

    public void setRaw(double rpm) {
        spark[0].getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
        spark[1].getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    public double getOutput() {
        return spark[0].getAppliedOutput();
    }
}
