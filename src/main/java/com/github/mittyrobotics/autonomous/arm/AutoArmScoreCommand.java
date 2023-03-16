package com.github.mittyrobotics.autonomous.arm;

import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoArmScoreCommand extends SequentialCommandGroup {
    public AutoArmScoreCommand(StateMachine.RobotState level, StateMachine.PieceState type) {
        super();

        addCommands(
                new AutoArmScorePart1(level, type),
                new AutoArmScorePart2(type)
        );
    }
}
