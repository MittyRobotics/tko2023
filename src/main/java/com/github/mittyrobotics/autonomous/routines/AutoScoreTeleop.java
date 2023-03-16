package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.SwerveAutoScoreCommand;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreTeleop extends SequentialCommandGroup {
    public AutoScoreTeleop(int index) {
        super();

//        LedSubsystem.getInstance().setAltColor(LedSubsystem.Color.BLUE);

        addCommands(
//                new InstantCommand(() -> LedSubsystem.getInstance().setAltColor(LedSubsystem.Color.BLUE))
              new SwerveAutoScoreCommand(Odometry.getInstance().getClosestScoringZone()[index], false)
        );

//        try {
//            Odometry.getInstance().getClosestScoringZone();
//            addCommands(
//                    new InstantCommand(() -> LedSubsystem.getInstance().setAltColor(LedSubsystem.Color.BLUE))
////                    new SwerveAutoScoreCommand(Odometry.getInstance().getClosestScoringZone()[index], false)
//            );
//        } catch (Exception ignored) {
//            System.out.println("\n\n\n\n\n\n\nTESTKHESJKHSEJKFSEF");
//        }
    }
}
