package com.github.mittyrobotics.intake.commands;

import com.github.mittyrobotics.intake.ClawGrabberSubsystem;
import com.github.mittyrobotics.intake.IntakeConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DropCommand extends InstantCommand {

    //drops game piece
    public DropCommand() {
        super(() -> ClawGrabberSubsystem.getInstance().setGrabberAngle(IntakeConstants.OPEN_DEGREES));
    }
}