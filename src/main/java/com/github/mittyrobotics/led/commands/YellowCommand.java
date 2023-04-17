package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class YellowCommand extends CommandBase {
    int[] yellowRgb;

    public YellowCommand() {
        super();
        addRequirements(LedSubsystem.getInstance());
        setName("yellow");
    }

    @Override
    public void initialize() {
        LedSubsystem.getInstance().startOutput();

        for (int i = 0; i < 3; i++) {
            yellowRgb[i] = LedConstants.RGB_VALUES[2][i];
        }
    }

    @Override
    public void execute() {
        LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                yellowRgb[0], yellowRgb[1], yellowRgb[2]);
    }

    @Override
    public void end(boolean interrupted) {
        LedSubsystem.getInstance().stopOutput();
    }

    @Override
    public boolean isFinished() {
        return OI.getInstance().getOperatorController().getLeftTriggerAxis() > 0.5;
    }
}

