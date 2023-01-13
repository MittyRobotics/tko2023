package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PurpleCommand extends CommandBase {

    int[] purpleHsv;

    public PurpleCommand() {
        addRequirements(LedSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LedSubsystem.getInstance().startOutput();

        for (int i = 0; i < 3; i++) {
            purpleHsv[i] = LedConstants.HSV_VALUES[6][i];
        }
    }

    @Override
    public void execute() {
        LedSubsystem.getInstance().setHsvRange(0, LedConstants.STRIP_LENGTH,
                purpleHsv[0], purpleHsv[1], purpleHsv[2]);
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
