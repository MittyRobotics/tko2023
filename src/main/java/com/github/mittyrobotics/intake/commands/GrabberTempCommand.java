package com.github.mittyrobotics.intake.commands;

import com.github.mittyrobotics.intake.ClawGrabberSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberTempCommand extends CommandBase {
    @Override
    public void initialize() {
        addRequirements(ClawGrabberSubsystem.getInstance());
    }

    @Override
    public void execute() {
        if (OI.getInstance().getOperatorController().getRightTriggerAxis() > 0.5) {
            ClawGrabberSubsystem.getInstance().open();
        } else {
            ClawGrabberSubsystem.getInstance().lock();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
