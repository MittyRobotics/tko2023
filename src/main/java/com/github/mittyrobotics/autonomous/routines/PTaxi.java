package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.PathFollowingCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.SwervePath;
import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PTaxi extends SequentialCommandGroup {
    public PTaxi(boolean low, boolean leftSide, StateMachine.PieceState piece) {
        super();

        int tag_id = leftSide ? (low ? 8 : 6) : (low ? 1 : 3);

        com.github.mittyrobotics.util.math.Pose p = Odometry.getInstance().getScoringZone(tag_id)[low ? 2 : 0];
        Pose scoring = new Pose(new Point(p.getPoint().getX(), p.getPoint().getY()), new Angle(p.getAngle().getRadians()));

        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                new Angle(leftSide ? 0 : Math.PI));

        Pose ending = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 100 : -100, 0)),
                new Angle(leftSide ? 0 : Math.PI));

        addCommands(
                new InitAutoCommand(new Pose(starting.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),

                new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),

                new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(starting, ending),
                                10, 3, 2, 2, 0, 2, true
                        ), leftSide ? Math.PI : 0, 5, 3,
                        0.2, 0.8, 0.5, 0, 0.01, true
                )
        );
    }
}