package com.github.mittyrobotics.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DropCommand extends InstantCommand {

    public DropCommand() {
        super(() -> ClawGrabberSubsystem.getInstance().setGrabberAngle(IntakeConstants.OPEN_DEGREES));
    }
}