package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RainbowEntireCommand extends CommandBase {

    double prevTime;

    public RainbowEntireCommand() {
        addRequirements(LedSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LedSubsystem.getInstance().startOutput();
        prevTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

        if(Timer.getFPGATimestamp() - prevTime > 0.5) {
            for (int i = 0; i < 6; i++) {
                LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_LENGTH,
                        LedConstants.RGB_VALUES[i][0], LedConstants.RGB_VALUES[i][1], LedConstants.RGB_VALUES[i][2]);
            }

            prevTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        LedSubsystem.getInstance().stopOutput();
    }

    @Override
    public boolean isFinished() {
        //TODO: Update
        return false;
    }
}
