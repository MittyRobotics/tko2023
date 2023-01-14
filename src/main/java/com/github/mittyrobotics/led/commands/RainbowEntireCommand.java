package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RainbowEntireCommand extends CommandBase {

    public RainbowEntireCommand() {
        addRequirements(LedSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LedSubsystem.getInstance().startOutput();
    }

    @Override
    public void execute() {
        for (int i = 0; i < 6; i++) {
            LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_LENGTH,
             LedConstants.RGB_VALUES[i][0], LedConstants.RGB_VALUES[i][1], LedConstants.RGB_VALUES[i][2]);
            try {
                Thread.sleep(1 * 1000);
            } catch (InterruptedException exception) {

            }
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
