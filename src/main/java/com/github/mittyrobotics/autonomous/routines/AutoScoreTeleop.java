package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.SwerveAutoScoreCommand;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreTeleop extends SequentialCommandGroup {
    public AutoScoreTeleop(int index) {
        super();

        addCommands(
              new SwerveAutoScoreCommand(Odometry.getInstance().getClosestScoringZone()[index], false)
        );
    }
}
