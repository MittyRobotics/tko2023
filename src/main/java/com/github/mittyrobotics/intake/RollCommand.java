package com.github.mittyrobotics.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RollCommand extends InstantCommand {

    //sets roller to either intake speed or 0 depending on whether a piece is held or not
    public RollCommand() {
        super(() -> ClawRollerSubsystem.getInstance().roll(ClawGrabberSubsystem.getInstance().pieceHeld() ? false : true));
    }
}
