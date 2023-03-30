package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand(int tag, int index, double maxvel, double maxaccel, double maxdecel, double startvel, double endvel) {
        super();

        //TODO TWO PATHS??? TINY LOOKAHEAD SECOND
        addCommands(
                new InstantCommand(() -> AutoPathManager.updateSplines(tag, index)),
                new ScorePFCommand(1,
                        new SwervePath(null, 4,
                                maxvel, maxaccel, maxdecel,
                                startvel, endvel, true
                        ),
                        0, 2, 0.02,
                        0, 1, 3, 0, 0, false
                )
                , new ScorePFCommand(2,
                        new SwervePath(null, 2,
                                maxvel, maxaccel, maxdecel,
                                maxvel, endvel, true
                        ),
                        0, 3, 0.02,
                        0, 1, 3, 0, 0, false
                )
        );
    }
}
