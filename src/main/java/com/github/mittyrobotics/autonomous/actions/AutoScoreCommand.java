package com.github.mittyrobotics.autonomous.actions;

import com.github.mittyrobotics.arm.StateMachine;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand() {
        super(
                new InstantCommand(StateMachine::handleHigh),
                new WaitCommand(0.3),
                new InstantCommand(StateMachine::handleScore),
                new WaitCommand(0.3),
                new InstantCommand(() -> { StateMachine.setCurrentArmState(StateMachine.ArmState.RETRACTED); }),
                new WaitCommand(0.3),
                new InstantCommand(StateMachine::handleStowed)
        );
    }
}
