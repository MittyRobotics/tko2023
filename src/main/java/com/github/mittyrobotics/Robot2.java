package com.github.mittyrobotics;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot2 extends TimedRobot {
    CANSparkMax spark;
    @Override
    public void robotInit() {
        spark = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
        spark.restoreFactoryDefaults();
        spark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        spark.getEncoder();
        spark.getEncoder().setPosition(0);
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        spark.set(0.25);
        System.out.println("Encoder: " + spark.getEncoder().getPosition() * 2.75);
    }

    @Override
    public void disabledInit() {
        spark.set(0);
    }
}
