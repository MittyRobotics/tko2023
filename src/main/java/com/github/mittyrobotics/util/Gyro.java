package com.github.mittyrobotics.util;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class Gyro {
    private static Gyro instance;

    public static Gyro getInstance() {
        if (instance == null) instance = new Gyro();
        return instance;
    }

    private WPI_Pigeon2 gyro;

    public Gyro() {
    }

    public void initHardware() {
        gyro = new WPI_Pigeon2(62);
    }

    public Angle getRadians() {
        return new Angle(-gyro.getAngle(), false);
    }

    public Angle getStandardized() {
        return new Angle(((getRadians().getRadians() % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI), true);
    }
}
