package com.github.mittyrobotics.pivot;

import com.github.mittyrobotics.pivot.commands.PivotToKinematics;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
            spark[i].getPIDController().setSmartMotionMaxAccel(Math.PI/12 / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO / (2 * Math.PI) * 60, 0);
            spark[i].getPIDController().setSmartMotionMaxVelocity(Math.PI/6 / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO / (2 * Math.PI) * 60, 0);
            spark[i].getPIDController().setFeedbackDevice(spark[i].getEncoder());
            spark[i].getPIDController().setFF(1/6000.);
            spark[i].getPIDController().setOutputRange(-0.1, 0.1);
//            spark[i].setClosedLoopRampRate(1.5);
            spark[i].getPIDController().setSmartMotionAllowedClosedLoopError(1, 0);
        }

        halifaxTop = new DigitalInput(PivotConstants.HALIFAX_TOP_CHANNEL);
        halifaxBottom = new DigitalInput(PivotConstants.HALIFAX_BOTTOM_CHANNEL);

//        setDefaultCommand(new PivotToKinematics());
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
       spark[0].getPIDController().setReference((radiansPerSecond / (2 * Math.PI / 60) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO) * 10, CANSparkMax.ControlType.kVelocity);
       spark[1].getPIDController().setReference((radiansPerSecond / (2 * Math.PI / 60) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO) * 10, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelocityDegreesPerSecond(double degreesPerSecond) {
        setVelocityRadiansPerSecond((degreesPerSecond * Math.PI / 180) * 10);
    }
// 25 in/s, 90 deg/s
    public double getPositionRadians() {
        return (spark[0].getEncoder().getPosition() * 2 * Math.PI) * PivotConstants.PIVOT_TO_NEO_GEAR_RATIO;
    }

    public double getPositionDegrees() {
        return getPositionRadians() * 180 / (2 * Math.PI);
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

    public boolean withinThreshold() {
        return Math.abs(ArmKinematics.getPivotDesiredCartesian().getRadians() - getPositionRadians()) < PivotConstants.PIVOT_THRESHOLD;
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

    public double getOutput() {
        return spark[0].getAppliedOutput();
    }
}
