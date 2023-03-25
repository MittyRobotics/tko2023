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
//
        int ind;
        if (index == 0) ind = Odometry.getInstance().FIELD_LEFT_SIDE ? 0 : 2;
        else if (index == 2) ind = Odometry.getInstance().FIELD_LEFT_SIDE ? 2 : 0;
        else ind = 1;

        addCommands(
                new TeleopScoreCommand(
                        Odometry.getInstance().getClosestScoringZone(
                                Odometry.getInstance().getState(), ind
                        ), index
                )
//              new OldAutoScoreCommand(Odometry.getInstance().getClosestScoringZone(Odometry.getInstance().getState(), ind)[ind], false)
        );
    }
}
