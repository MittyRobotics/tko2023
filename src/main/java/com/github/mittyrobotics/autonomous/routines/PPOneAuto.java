package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.arm.StateMachine;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.actions.AutoScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.AutoPathManager;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.math.Pose;
import com.github.mittyrobotics.util.math.autonomous.QuinticHermiteSpline;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static java.lang.Math.*;

public class PPOneAuto extends SequentialCommandGroup {
    Pose start = Odometry.getInstance().getState();

    public PPOneAuto() {
        super();
        addCommands(
                new AutoScoreCommand(),
                new InstantCommand(() -> { AutoPathManager.setCurrentPath(new SwervePath(
                        new QuinticHermiteSpline(start, new Pose(0, 0, 0, true)),
                        new Angle(0, true), new Angle(PI, true),
                        0, 0, 4, 5, 5, 0,
                        0.8, 0.4, 1, 1,
                        0.6, 0, 0
                        )); }
                ),
                new InstantCommand(() -> StateMachine.setPieceState(StateMachine.PieceState.CONE)),
                new InstantCommand(StateMachine::handleLow),
                new InstantCommand(() -> StateMachine.setIntakeState(StateMachine.IntakeState.INTAKING))
        );
    }
}
