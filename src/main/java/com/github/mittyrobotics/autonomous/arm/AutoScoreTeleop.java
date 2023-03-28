package com.github.mittyrobotics.autonomous.arm;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.OldAutoScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.TeleopScoreCommand;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreTeleop extends SequentialCommandGroup {
    public AutoScoreTeleop(int index) {
        super();

        addRequirements(SwerveSubsystem.getInstance());

        addCommands(
                new TeleopScoreCommand(
                        Odometry.getClosestScoringZone(
                                Odometry.getInstance().getState()
                        ), index
                )
//              new OldAutoScoreCommand(Odometry.getInstance().getClosestScoringZone(Odometry.getInstance().getState(), ind)[ind], false)
        );
    }
}
