package com.github.mittyrobotics.autonomous.arm;

import com.github.mittyrobotics.autonomous.pathfollowing.OldAutoScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.AutoScoreCommand;
import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreCommandGroup extends SequentialCommandGroup {

    public AutoScoreCommandGroup(int target, int index, StateMachine.RobotState level, StateMachine.PieceState piece,
                                 double maxvel, double maxaccel, double maxdecel, double startvel, double endvel) {
        super();
        addCommands(
                new AutoArmScorePart1(level, piece),
                new AutoScoreCommand(target, index, maxvel, maxaccel, maxdecel, startvel, endvel),
                new AutoArmScorePart2(piece)
        );
    }
}
