package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TeleopScoreCommand extends SequentialCommandGroup {


    public TeleopScoreCommand(int index) {
        super();

        addCommands(
            new InstantCommand(() -> AutoPathManager.updateSplines(-1, index)),
            new ScorePFCommand(1,
                    new SwervePath(null, 6,
                            0, 0, 0,
                            0, 0, false
                    ),
                    0, 3, 0.02,
                    0, 1, 3, 0, 0, false
            ), new ScorePFCommand(2,
                    new SwervePath(null, 2,
                            0, 0, 0,
                            0, 0, false
                    ),
                    0, 3, 0.02,
                    0, 1, 3, 0, 0, false
            )
        );
    }
}
