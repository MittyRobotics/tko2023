package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.AutoLineDrive;
import com.github.mittyrobotics.autonomous.pathfollowing.OldSwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PreloadAndBalanceAuto extends SequentialCommandGroup {
    public PreloadAndBalanceAuto(boolean leftSide) {
        super();

        //tag should be 1, 2, 3 from bottom to top
        Pose scoring = Odometry.getInstance().getScoringZone(leftSide ? 7 : 2)[0];
        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                scoring.getHeading());

        addCommands(
                new InitAutoCommand(starting),
                new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),
                new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),

                new Balance(false)
        );

    }
}
