package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RainbowGroupCommand extends CommandBase {

    int block;

    public RainbowGroupCommand() {
        addRequirements(LedSubsystem.getInstance());
    }

    @Override
    public void initialize() {

        LedSubsystem.getInstance().startOutput();
    }

    @Override
    public void execute() {

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
