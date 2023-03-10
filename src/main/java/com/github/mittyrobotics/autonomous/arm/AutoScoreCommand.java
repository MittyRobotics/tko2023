package com.github.mittyrobotics.autonomous.arm;

import com.github.mittyrobotics.autonomous.pathfollowing.SwerveAutoScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreCommand extends SequentialCommandGroup {
    private StateMachine.RobotState level;
    private boolean auto;

    public AutoScoreCommand(Pose target, StateMachine.RobotState level, boolean auto) {
        super();
        this.level = level;
        addCommands(
                new SwerveAutoScoreCommand(target),
                new AutoArmScoreCommand(level, auto)
        );
    }
}
