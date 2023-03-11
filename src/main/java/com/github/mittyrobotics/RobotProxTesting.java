package com.github.mittyrobotics;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;

public class RobotProxTesting extends TimedRobot {
    DigitalInput prox;

    @Override
    public void robotInit() {
        prox = new DigitalInput(6);
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        System.out.println(prox.get());
    }
}
