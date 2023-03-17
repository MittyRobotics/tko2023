package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.AutoLineDrive;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.swing.plaf.nimbus.State;

public class PreloadAndBalanceAuto extends SequentialCommandGroup {
    public PreloadAndBalanceAuto(boolean leftSide) {
        super();

        Pose scoring = Odometry.getInstance().getScoringZone(leftSide ? 8 : 1)[2];
        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                scoring.getHeading());

        Pose mid = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 160 : -160, 0)),
                starting.getHeading());

        Pose mid_up = new Pose(Point.add(mid.getPosition(), new Point(0, 66)), starting.getHeading());

        Pose balance = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 80 : -80,
                66)), starting.getHeading());

        addCommands(
                new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                        Odometry.getInstance().FIELD_LEFT_SIDE ? 1 : 2
                )),
                new InitAutoCommand(starting),
                new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),
                new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                            new QuinticHermiteSpline(starting, mid),
                            starting.getHeading(), mid.getHeading(),
                            0, 0, 3, 3, 2,
                            0, 0, 2.5, 0, 0.02, 0.5
                    )
                ),
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(mid, mid_up),
                                mid.getHeading(), mid_up.getHeading(),
                                0, 0, 3, 3, 2,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),
                new AutoBalanceCommand(3, 1.5)
//                new AutoLineDrive(2, 0.05,
//                        new SwervePath(
//                                new QuinticHermiteSpline(mid_up, balance),
//                                mid_up.getHeading(), balance.getHeading(),
//                                0, 0, 0.8, 1, 0.8,
//                                0, 0, 2.5, 0, 0.02, 0.5
//                        )
//                ),
//                new InstantCommand(() -> SwerveSubsystem.getInstance().fortyFiveAngle())
        );
    }
}
