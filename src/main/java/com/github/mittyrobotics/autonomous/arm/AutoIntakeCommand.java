package com.github.mittyrobotics.autonomous.arm;

import com.github.mittyrobotics.autonomous.pathfollowing.AutoPickupCommand;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoIntakeCommand extends SequentialCommandGroup {
    public AutoIntakeCommand(StateMachine.RobotState level, StateMachine.PieceState piece) {
        super();
        addCommands(
                new InstantCommand(() -> {
                    if (level == StateMachine.RobotState.GROUND) {
                        OI.getInstance().handleGround();
                    } else if (level == StateMachine.RobotState.HP) {
                        OI.getInstance().handleHumanPlayer();
                    }
                }),
                new AutoPickupCommand(
                        piece == StateMachine.PieceState.CONE, 0, true
                )
        );
    }
}
