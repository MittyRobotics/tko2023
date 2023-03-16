package com.github.mittyrobotics.autonomous.arm;

import com.github.mittyrobotics.autonomous.pathfollowing.AutoScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreCommandGroup extends SequentialCommandGroup {

    public AutoScoreCommandGroup(Pose target, StateMachine.RobotState level, StateMachine.PieceState piece) {
        super();
        addCommands(
                new AutoScoreCommand(target, true),
                new AutoArmScoreCommand(level, piece, true)
        );
    }
}
