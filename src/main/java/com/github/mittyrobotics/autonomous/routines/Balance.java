package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Balance extends SequentialCommandGroup {
    public Balance(boolean initialDirectionForward) {
        super();

        addCommands(
                new InstantCommand(() -> SwerveSubsystem.getInstance().setRampRate(0)),
                new TimedBangBangBalance(2.5, 650, 0.26, initialDirectionForward)
//                new FastOvershootBalance(3, 0.7, initialDirectionForward)
        );
    }
}
