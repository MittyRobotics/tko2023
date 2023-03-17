package com.github.mittyrobotics.autonomous.arm;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.AutoScoreCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreTeleop extends SequentialCommandGroup {
    public AutoScoreTeleop(int index) {
        super();

        if (index == 0) index = Odometry.getInstance().FIELD_LEFT_SIDE ? 0 : 2;
        if (index == 2) index = Odometry.getInstance().FIELD_LEFT_SIDE ? 2 : 0;

        addCommands(
              new AutoScoreCommand(Odometry.getInstance().getClosestScoringZone(Odometry.getInstance().getState(), index)[index], false)
        );
    }
}
