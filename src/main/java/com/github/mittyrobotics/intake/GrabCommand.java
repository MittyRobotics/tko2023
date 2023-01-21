package com.github.mittyrobotics.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class GrabCommand extends InstantCommand {

    public GrabCommand(boolean isCone) {
        super(() -> ClawGrabberSubsystem.getInstance().setGrabberAngle(isCone ? IntakeConstants.CONE_DEGREES : IntakeConstants.CUBE_DEGREES));
    }
}
