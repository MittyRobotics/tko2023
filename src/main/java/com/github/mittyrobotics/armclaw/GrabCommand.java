package com.github.mittyrobotics.armclaw;

import com.github.mittyrobotics.util.OI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class GrabCommand extends InstantCommand {

    public GrabCommand(boolean isCone) {
        super(() -> ClawGrabberSubsystem.getInstance().setGrabberAngle(isCone ? ClawConstants.CONE_DEGREES : ClawConstants.CUBE_DEGREES));
    }
}
