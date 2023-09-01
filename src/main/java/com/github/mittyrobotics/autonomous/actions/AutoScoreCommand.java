package com.github.mittyrobotics.autonomous.actions;

import com.github.mittyrobotics.arm.StateMachine;
import edu.wpi.first.wpilibj2.command.*;

import javax.swing.plaf.nimbus.State;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand() {
        super(
                // TODO: 9/1/2023 ADD CONE VS CUBE MODE 
                new InstantCommand(StateMachine::handleHigh),
                new WaitUntilCommand(StateMachine::withinThreshold),
                new WaitCommand(0.3),
                
                new InstantCommand(StateMachine::handleScore),
                new WaitUntilCommand(StateMachine::withinThreshold),
                new WaitCommand(0.3),
                
                new InstantCommand(() -> StateMachine.setIntakeState(StateMachine.IntakeState.OUTTAKING)),
                new WaitCommand(0.05),
                new InstantCommand(() -> StateMachine.setArmState(StateMachine.ArmState.RETRACTED)),
                new WaitCommand(0.3),

                new WaitUntilCommand(StateMachine::withinThreshold),
                new InstantCommand(() -> StateMachine.setPieceState(StateMachine.PieceState.NONE))
        );
    }
}
