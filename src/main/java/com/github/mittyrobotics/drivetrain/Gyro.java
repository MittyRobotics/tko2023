package com.github.mittyrobotics.drivetrain;



import com.ctre.phoenix6.hardware.Pigeon2;
import com.github.mittyrobotics.util.interfaces.IHardware;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.github.mittyrobotics.util.interfaces.IMotorSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase implements IHardware {

    public static Gyro instance;

    private Pigeon2 gyro;

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro();
        }
        return instance;
    }

    @Override
    public void initHardware() {
        gyro = new Pigeon2(0);
    }

    public double getRadians() {
        return gyro.getAngle();
    }

}
