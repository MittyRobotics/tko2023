package com.github.mittyrobotics.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeCommand extends SequentialCommandGroup {
    //intakes based on cone or cube
    public IntakeCommand(boolean isCone){
        super(
                new GrabCommand(isCone), new RollCommand()
        );
    }

}