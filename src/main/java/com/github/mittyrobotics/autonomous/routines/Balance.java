package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Balance extends SequentialCommandGroup {
    public Balance(boolean initialDirectionForward) {
        super();

        addCommands(
                new InstantCommand(() -> SwerveSubsystem.getInstance().setRampRate(0.1)),
                new FastOvershootBalance(3, 0.7, initialDirectionForward)
        );
    }
}
