package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class YellowCommand extends CommandBase {

    int[] yellowHsv;

    public YellowCommand() {
        addRequirements(LedSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LedSubsystem.getInstance().startOutput();

        for (int i = 0; i < 3; i++) {
            yellowHsv[i] = LedConstants.HSV_VALUES[2][i];
        }
    }

    @Override
    public void execute() {
        LedSubsystem.getInstance().setHsvRange(0, LedConstants.STRIP_LENGTH,
                yellowHsv[0], yellowHsv[1], yellowHsv[2]);
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
