package com.github.mittyrobotics.armclaw;

import com.github.mittyrobotics.util.OI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DropCommand extends InstantCommand {

    public DropCommand() {
        super(() -> ClawGrabberSubsystem.getInstance().setGrabberAngle(ClawConstants.OPEN_DEGREES));
    }
}