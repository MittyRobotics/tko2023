package com.github.mittyrobotics;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;

public class RobotTest extends Robot {

    PWM pwm;
    @Override
    public void robotInit() {
        pwm = new PWM(5);
    }

    @Override
    public void robotPeriodic() {
        pwm.setRaw(5);
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
    }
}
