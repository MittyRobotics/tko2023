package com.github.mittyrobotics.autonomous.arm;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoArmScorePart1 extends InstantCommand {
    public AutoArmScorePart1(StateMachine.RobotState level, StateMachine.PieceState type) {
        super(() -> {
            if(type == StateMachine.PieceState.CONE)
                StateMachine.getInstance().setStateCone();
            if(type == StateMachine.PieceState.CUBE)
                StateMachine.getInstance().setStateCube();

            if (level == StateMachine.RobotState.MID) {
                OI.getInstance().handleMid();
            } else {
                OI.getInstance().handleHigh();
            }
        });
    }
}
