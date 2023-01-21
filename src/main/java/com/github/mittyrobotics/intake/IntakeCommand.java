package com.github.mittyrobotics.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(boolean isCone){
        super(
                new GrabCommand(isCone), new RollCommand()
        );
    }

}
