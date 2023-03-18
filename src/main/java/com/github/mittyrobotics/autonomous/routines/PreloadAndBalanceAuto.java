package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.AutoLineDrive;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PreloadAndBalanceAuto extends SequentialCommandGroup {
    public PreloadAndBalanceAuto(boolean leftSide, int tag, int index, boolean balance) {
        super();

        //tag should be 1, 2, 3 from bottom to top
        Pose scoring = Odometry.getInstance().getScoringZone(leftSide ? 9 - tag : tag)[index];
        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                scoring.getHeading());

        Pose before_mid = new Pose(new Point(starting.getPosition().getX() + (leftSide ? 15 : -15),
                tag == 1 ? 33 : 183),
                scoring.getHeading());

        Pose mid = new Pose(new Point(starting.getPosition().getX() + (leftSide ? 160 : -160),
                before_mid.getPosition().getY()),
                starting.getHeading());

        Pose mid_up = new Pose(Point.add(mid.getPosition(), new Point(0, (tag == 1 ? 66 : -66))),
                starting.getHeading());

        int cam = Odometry.getInstance().FIELD_LEFT_SIDE ?
                (index == 2 ? 1 : 2) :
                (index == 2 ? 2 : 1);

//        Pose balance = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 80 : -80,
//                66)), starting.getHeading());

        if (balance) {
            if (tag == 1 || tag == 3) {

                addCommands(
                        new InstantCommand(() -> Odometry.getInstance().setCustomCam(cam)),
                        new InitAutoCommand(starting),
                        new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),
                        new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),
                        new AutoLineDrive(4, 0.05,
                                new SwervePath(
                                        new QuinticHermiteSpline(starting, before_mid),
                                        starting.getHeading(), before_mid.getHeading(),
                                        0, 2, 3, 3, 2,
                                        0, 0, 2.5, 0, 0.02, 0.5
                                )
                        ),
                        new AutoLineDrive(4, 0.05,
                                new SwervePath(
                                        new QuinticHermiteSpline(before_mid, mid),
                                        before_mid.getHeading(), mid.getHeading(),
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
                        new FastOvershootBalance(3, 1.5, true)
                );

            } else if (tag == 2) {

                addCommands(
                        new InstantCommand(() -> Odometry.getInstance().setCustomCam(cam)),
                        new InitAutoCommand(starting),
                        new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),
                        new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),

                        new FastOvershootBalance(3.5, 0.5, false)
                );

            }
        } else {
            if (tag == 1 || tag == 3) {
                addCommands(
                        new InstantCommand(() -> Odometry.getInstance().setCustomCam(cam)),
                        new InitAutoCommand(starting),
                        new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),
                        new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),
                        new AutoLineDrive(4, 0.05,
                                new SwervePath(
                                        new QuinticHermiteSpline(starting, before_mid),
                                        starting.getHeading(), before_mid.getHeading(),
                                        0, 2, 3, 3, 2,
                                        0, 0, 2.5, 0, 0.02, 0.5
                                )
                        ),
                        new AutoLineDrive(4, 0.05,
                                new SwervePath(
                                        new QuinticHermiteSpline(before_mid, mid),
                                        before_mid.getHeading(), mid.getHeading(),
                                        0, 0, 3, 3, 2,
                                        0, 0, 2.5, 0, 0.02, 0.5
                                )
                        )
                );
            } else if (tag == 2) {
                addCommands(
                        new InstantCommand(() -> Odometry.getInstance().setCustomCam(cam)),
                        new InitAutoCommand(starting),
                        new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),
                        new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE)
                );
            }
        }
    }
}
