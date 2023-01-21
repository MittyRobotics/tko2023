package com.github.mittyrobotics.intake.commands;

import com.github.mittyrobotics.intake.ClawGrabberSubsystem;
import com.github.mittyrobotics.intake.IntakeConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class GrabCommand extends InstantCommand {

    //picks up game piece with claws, sets grabber subsystem to cone or cube depending on input boolean
    public GrabCommand(boolean isCone) {
        super(() -> {
            ClawGrabberSubsystem.getInstance().setGrabberAngle(isCone ? IntakeConstants.CONE_DEGREES : IntakeConstants.CUBE_DEGREES);
            ClawGrabberSubsystem.getInstance().setConeOrCube(isCone);}
        );

    }
}
