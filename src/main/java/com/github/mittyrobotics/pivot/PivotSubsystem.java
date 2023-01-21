package com.github.mittyrobotics.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    public static PivotSubsystem instance;
    private CANSparkMax spark;

    public static PivotSubsystem getInstance() {
        return instance == null ? new PivotSubsystem() : instance;
    }

    private PivotSubsystem() {

    }

    public void initHardware() {
        spark = new CANSparkMax(PivotConstants.PIVOT_ID, CANSparkMax.MotorType.kBrushless);
        spark.restoreFactoryDefaults();
        spark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        spark.getPIDController().setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
        spark.getPIDController().setSmartMotionMaxAccel(PivotConstants.MAX_ACCEL, 0);
        spark.getPIDController().setSmartMotionMaxAccel(PivotConstants.MAX_VEL, 0);
        spark.getPIDController().setFeedbackDevice(spark.getEncoder());
    }

    public void setPositionRadians(double radians) {
        spark.getPIDController().setReference(radians / (2 * Math.PI) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setPositionDegrees(double degrees) {
        setPositionRadians(Math.PI / 180 * degrees);
    }

    public void setVelocityRadiansPerSecond(double radiansPerSecond) {
        spark.getPIDController().setReference(radiansPerSecond / (2 * Math.PI / 60) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelocityDegreesPerSecond(double degreesPerSecond) {
        setVelocityRadiansPerSecond(degreesPerSecond * Math.PI / 180);
    }

    public double getPositionRadians() {
        return spark.getEncoder().getPosition() * 2 * Math.PI;
    }

    public double getPositionDegrees() {
        return getPositionRadians() * 180 / (2 * Math.PI);
    }

    public double getVelocityRadiansPerSecond() {
        return spark.getEncoder().getVelocity() * 2 * Math.PI / 60;
    }

    public double getVelocityDegreesPerSecond() {
        return getVelocityRadiansPerSecond() * 180 / (2 * Math.PI);
    }

    public void configPID (double p, double i, double d) {
        spark.getPIDController().setP(p);
        spark.getPIDController().setI(i);
        spark.getPIDController().setD(d);
    }
}
