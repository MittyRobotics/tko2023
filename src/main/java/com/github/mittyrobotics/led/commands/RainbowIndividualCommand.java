package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RainbowIndividualCommand extends CommandBase {

    int cur;

    public RainbowIndividualCommand() {
        addRequirements(LedSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LedSubsystem.getInstance().startOutput();
        cur = 0;
    }

    @Override
    public void execute() {
        if(cur == 6) {
            cur = 0;
        }

        int prev = cur;

        for (int i = 0; i < 255; i++) {
            if(prev == 6) {
                prev = 0;
            }

            LedSubsystem.getInstance().setRgb(i, LedConstants.RGB_VALUES[prev][0], LedConstants.RGB_VALUES[prev][1],
                    LedConstants.RGB_VALUES[prev][2]);
            prev++;
        }

        cur++;

        try {
            Thread.sleep(1 * 1000);
        } catch (InterruptedException exception) {

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
