package com.github.mittyrobotics.autonomous.actions;

import com.github.mittyrobotics.arm.StateMachine;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand() {
        super(
                new InstantCommand(StateMachine)
        );
    }
}
