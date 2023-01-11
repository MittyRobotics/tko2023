package com.github.mittyrobotics.util;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.util.interfaces.IMotorSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase implements IMotorSubsystem {

    private static Gyro instance;

    WPI_Pigeon2 gyro;

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


    @Override
    public void updateDashboard() {

    }

    @Override
    public void initHardware() {
        gyro = new WPI_Pigeon2(SwerveConstants.PIGEON_TWO_ID);
        gyro.configFactoryDefault();
        gyro.reset();
    }

    @Override
    public void overrideSetMotor(double percent) {

    }

    public double getHeadingAngle() {
        return gyro.getAngle();
    }

    public double getHeadingRadians() {
//        System.out.println(getHeadingAngle() * Math.PI / 180.);
        return -getHeadingAngle() * Math.PI / 180.;
    }
}
