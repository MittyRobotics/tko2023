package com.github.mittyrobotics;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.TimedRobot;

public class RobotFFTest extends TimedRobot {
    CANSparkMax spark;
    protected RobotFFTest() {
        super();
    }

    @Override
    public void robotInit() {
        spark = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void teleopInit() {
        spark.set(1);
    }

    @Override
    public void robotPeriodic() {
        System.out.println("Vel: " + spark.getEncoder().getVelocity());


    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
    }

    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
    }
}
