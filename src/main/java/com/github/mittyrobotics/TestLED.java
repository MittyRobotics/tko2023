package com.github.mittyrobotics;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class TestLED extends TimedRobot {

    @Override
    public void robotInit() {
        LedSubsystem.getInstance().initHardware();
        OI.getInstance().setupTestLEDControls();

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        LedSubsystem.getInstance().stopOutput();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopPeriodic() {

    }


}
