package com.github.mittyrobotics.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RollCommand extends InstantCommand {
    public RollCommand() {
        super(() -> ClawRollerSubsystem.getInstance().roll(ClawGrabberSubsystem.getInstance().pieceHeld() ? false : true));
    }
}
