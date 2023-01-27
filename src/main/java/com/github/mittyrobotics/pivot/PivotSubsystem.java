package com.github.mittyrobotics.pivot;

import com.github.mittyrobotics.pivot.commands.PivotToKinematics;
import com.github.mittyrobotics.telescope.TelescopeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private static PivotSubsystem instance;
    private CANSparkMax[] spark = new CANSparkMax[2];
    private DigitalInput halifax;

    public static PivotSubsystem getInstance() {
        return instance == null ? new PivotSubsystem() : instance;
    }

    private PivotSubsystem() {

    }

    public void initHardware() {
        for (int i = 0; i < 2; i++) {
            spark[i] = new CANSparkMax(PivotConstants.PIVOT_ID, CANSparkMax.MotorType.kBrushless);
            spark[i].restoreFactoryDefaults();
            spark[i].setIdleMode(CANSparkMax.IdleMode.kBrake);
            spark[i].getPIDController().setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
            spark[i].getPIDController().setSmartMotionMaxAccel(PivotConstants.MAX_ACCEL, 0);
            spark[i].getPIDController().setSmartMotionMaxAccel(PivotConstants.MAX_VEL, 0);
            spark[i].getPIDController().setFeedbackDevice(spark[i].getEncoder());
        }

        halifax = new DigitalInput(PivotConstants.HALIFAX_ID);

        setDefaultCommand(new PivotToKinematics());
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

    public void setVelocityRadiansPerSecond(double radiansPerSecond) {
        spark[0].getPIDController().setReference(radiansPerSecond / (2 * Math.PI / 60) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, CANSparkMax.ControlType.kVelocity);
        spark[1].getPIDController().setReference(radiansPerSecond / (2 * Math.PI / 60) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelocityDegreesPerSecond(double degreesPerSecond) {
        setVelocityRadiansPerSecond(degreesPerSecond * Math.PI / 180);
    }

    public double getPositionRadians() {
        return (spark[0].getEncoder().getPosition() * 2 * Math.PI) * PivotConstants.PIVOT_TO_NEO_GEAR_RATIO;
    }

    public double getPositionDegrees() {
        return getPositionRadians() * 180 / (2 * Math.PI);
    }

    public double getVelocityRadiansPerSecond() {
        return (spark[0].getEncoder().getVelocity() * 2 * Math.PI / 60) * PivotConstants.PIVOT_TO_NEO_GEAR_RATIO;
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
        return Math.abs(ArmKinematics.getPivotDesired().getRadians() - getPositionRadians()) < PivotConstants.PIVOT_THRESHOLD;
    }

    public void resetAngleDegrees(double degrees) {
        spark[0].getEncoder().setPosition(degrees / 360);
        spark[1].getEncoder().setPosition(degrees / 360);
    }

    public void resetAngleRadians(double radians) {
        spark[0].getEncoder().setPosition(radians / (2*Math.PI));
        spark[1].getEncoder().setPosition(radians / (2*Math.PI));
    }

    public boolean getHalifaxContact() {
        return !halifax.get();
    }
}
