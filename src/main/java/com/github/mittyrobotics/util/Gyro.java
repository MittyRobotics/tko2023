package com.github.mittyrobotics.util;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.util.interfaces.IMotorSubsystem;
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

    public void setAngleOffset(double offset) {
        angleOffset = new Angle(offset);
    }

    public Angle getAngleOffset() {
        return angleOffset;
    }

    public double getHeadingAngle() {
        return gyro.getAngle();
    }

    public double getHeadingRadians() {
//        System.out.println(getHeadingAngle() * Math.PI / 180.);
        return -getHeadingAngle() * Math.PI / 180. + (angleOffset == null ? 0 : angleOffset.getRadians());
    }

    public double getHeadingRadiansNoOffset() {
        return -getHeadingAngle() * Math.PI / 180.;
    }

    public double getAngularVel() {
        return gyro.getRate() * Math.PI / 180;
    }
}
