package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.arm.AutoScoreCommandGroup;
import com.github.mittyrobotics.autonomous.pathfollowing.AutoLineDrive;
import com.github.mittyrobotics.autonomous.pathfollowing.AutoScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PlusOneConeAuto extends SequentialCommandGroup {
    public PlusOneConeAuto(boolean leftSide) {
        super();

        Pose scoring = Odometry.getInstance().getScoringZone(leftSide ? 8 : 1)[2];
        Pose scoring_second = Odometry.getInstance().getScoringZone(leftSide ? 8 : 1)[0];

        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                scoring.getHeading());

        Pose firstCone = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 180 : -180,15)),
                new Angle(leftSide ? 0 : Math.PI));

        Pose beforeFirstCone = new Pose(Point.add(firstCone.getPosition(), new Point(leftSide ? -80 : 80, 0)),
                scoring.getHeading());

        Pose beforeAutoScore = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 60 : -60, 15)),
                starting.getHeading());

        Pose mid = new Pose(Point.add(beforeAutoScore.getPosition(), new Point(leftSide ? 100 : -100, 0)),
                starting.getHeading());

        Pose mid_up = new Pose(Point.add(mid.getPosition(), new Point(0, 60)), starting.getHeading());

        Pose balance = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 80 : -80,
                60)), starting.getHeading());

        addCommands(
                new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),
                new InitAutoCommand(starting),
                new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(starting, beforeFirstCone),
                                starting.getHeading(), beforeFirstCone.getHeading(),
                                0, 5, 5, 3, 3,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),
                new InstantCommand(() -> OI.getInstance().handleGround()),
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(beforeFirstCone, firstCone),
                                beforeFirstCone.getHeading(), firstCone.getHeading(),
                                0, 0, 3, 3, 1.5,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),
                new InstantCommand(() -> {
                    OI.getInstance().zeroAll();
                    StateMachine.getInstance().setIntakeStowing();
                    Odometry.getInstance().setScoringCam(true);
                }),
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(firstCone, beforeAutoScore),
                                firstCone.getHeading(), beforeAutoScore.getHeading(),
                                0, 1.5, 3, 3, 3,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),
                new AutoScoreCommandGroup(scoring_second, StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(scoring, beforeAutoScore),
                                beforeAutoScore.getHeading(), beforeAutoScore.getHeading(),
                                0, 3, 5, 5, 3,
                                0, 0, 0.5, 0, 0.02, 0.5
                        )
                ),
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(beforeAutoScore, mid),
                                beforeAutoScore.getHeading(), mid.getHeading(),
                                0, 3, 5, 5, 5,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(mid, mid_up),
                                mid.getHeading(), mid_up.getHeading(),
                                0, 3, 5,  5, 5,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),
                new AutoLineDrive(2, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(mid_up, balance),
                                mid_up.getHeading(), balance.getHeading(),
                                0, 0, 5, 5, 5,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),
                new InstantCommand(() -> SwerveSubsystem.getInstance().fortyFiveAngle())
        );
    }
}
