package com.github.mittyrobotics.util;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.github.mittyrobotics.util.math.*;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.util.math.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {

    private static Gyro instance;

    WPI_Pigeon2 gyro;
    Angle angleOffset = null;

    public short[] pose = new short[3];

    public Gyro() {
        super();
        setName("Gyro");
    }

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro();
        }
        return instance;
    }

    public void initHardware() {
        gyro = new WPI_Pigeon2(62);
        gyro.configFactoryDefault();
        gyro.reset();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public void reset() {
        gyro.reset();
    }

    public void setAngleOffset(double offset, boolean radians) {
        angleOffset = new Angle(offset, radians);
    }

    public Angle getAngleOffset() {
        return angleOffset;
    }

    public double getHeadingAngleRaw() {
        return gyro.getAngle();
    }

    public double getHeadingRadians() {
//        System.out.println(getHeadingAngle() * Math.PI / 180.);
        return getHeadingRadiansNoOffset() + (angleOffset == null ? 0 : angleOffset.getRadians());
    }

    public Angle getRadiansAsAngle() {
        return new Angle(getHeadingRadians(), true);
    }

    public double getHeadingRadiansNoOffset() {
        return -getHeadingAngleRaw() * Math.PI / 180.;
    }

    public double getAngularVel() {
        return gyro.getRate() * Math.PI / 180;
    }
}
