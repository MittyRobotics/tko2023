package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.arm.AutoScoreCommandGroup;
import com.github.mittyrobotics.autonomous.pathfollowing.AutoLineDrive;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class HighPlusConeCubeAuto extends SequentialCommandGroup {
    public HighPlusConeCubeAuto(boolean leftSide, boolean balance) {

        Pose scoring = Odometry.getInstance().getScoringZone(leftSide ? 6 : 3)[0];
        Pose scoring_second = Odometry.getInstance().getScoringZone(leftSide ? 6 : 3)[2];
        Pose scoring_third = Odometry.getInstance().getScoringZone(leftSide ? 6 : 3)[1];

        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                scoring.getHeading());

        Pose beforeTurnAround = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 30 : -30, -10)),
                scoring.getHeading());

        Pose firstCone = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 195 : -195,-10)),
                new Angle(leftSide ? 0 : Math.PI));

        Pose beforeFirstCone = new Pose(Point.add(firstCone.getPosition(), new Point(leftSide ? -105 : 105, 0)),
                firstCone.getHeading());

        Pose beforeAutoScore = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 30 : -30, -5)),
                starting.getHeading());

        Pose beforeSecondPickup = new Pose(Point.add(beforeAutoScore.getPosition(), new Point(leftSide ? 120 : -120, 0)),
                starting.getHeading());

        Pose secondPickup = new Pose(Point.add(beforeSecondPickup.getPosition(), new Point(leftSide ? 43 : -43, -35))
                , new Angle(leftSide ? 5.57 : -2.43));

        addCommands(
                // FIRST CONE
                new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                        Odometry.getInstance().FIELD_LEFT_SIDE ? 3 : 0 //left vs right BACK cam
                )),
                new InitAutoCommand(starting),
                new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),

                new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),

                // DRIVE BACK & TURN AROUND
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(starting, beforeTurnAround),
                                starting.getHeading(), beforeTurnAround.getHeading(),
                                0, 3, 3, 8, 5,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(beforeTurnAround, beforeFirstCone),
                                beforeTurnAround.getHeading(), beforeFirstCone.getHeading(),
                                0, 3, 3, 8, 5,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),

                new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                        Odometry.getInstance().FIELD_LEFT_SIDE ? 2 : 1 //left vs right FRONT cam
                )),

                // INTAKE
                new InstantCommand(() -> OI.getInstance().handleGround()),
                new InstantCommand(() -> StateMachine.getInstance().setStateCone()),
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(beforeFirstCone, firstCone),
                                beforeFirstCone.getHeading(), firstCone.getHeading(),
                                0, 0, 3, 8, 5,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),
                new InstantCommand(() -> {
                    OI.getInstance().zeroAll();
                    StateMachine.getInstance().setIntakeStowing();
                    Odometry.getInstance().setScoringCam(true);
                }),

                // GO BACK AND AUTOSCORE
                new AutoLineDrive(4, 0.05,
                        new SwervePath(
                                new QuinticHermiteSpline(firstCone, beforeAutoScore),
                                firstCone.getHeading(), beforeAutoScore.getHeading(),
                                0, 1, 3, 5, 5,
                                0, 0, 2.5, 0, 0.02, 0.5
                        )
                ),
                new AutoScoreCommandGroup(scoring_second, StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE)

//
//                new AutoLineDrive(4, 0.05,
//                        new SwervePath(
//                                new QuinticHermiteSpline(starting, beforeAutoScore),
//                                starting.getHeading(), beforeAutoScore.getHeading(),
//                                0, 3, 3, 5, 5,
//                                0, 0, 2.5, 0, 0.02, 0.5
//                        )
//                ),
//
//                new AutoLineDrive(4, 0.05,
//                        new SwervePath(
//                                new QuinticHermiteSpline(beforeAutoScore, beforeSecondPickup),
//                                beforeAutoScore.getHeading(), beforeSecondPickup.getHeading(),
//                                0, 2, 3, 5, 5,
//                                0, 0, 2.5, 0, 0.02, 0.5
//                        )
//                ),
//
//                new InstantCommand(() -> OI.getInstance().handleGround()),
//                new InstantCommand(() -> StateMachine.getInstance().setStateCube()),
//
//                new AutoLineDrive(4, 0.05,
//                        new SwervePath(
//                                new QuinticHermiteSpline(beforeSecondPickup, secondPickup),
//                                beforeSecondPickup.getHeading(), secondPickup.getHeading(),
//                                0, 0, 3, 5, 5,
//                                0, 0, 2.5, 0, 0.02, 0.5
//                        )
//                ),
//
//                new InstantCommand(() -> {
//                    OI.getInstance().zeroAll();
//                    StateMachine.getInstance().setIntakeStowing();
//                    Odometry.getInstance().setScoringCam(true);
//                }),
//
//                new AutoLineDrive(4, 0.05,
//                        new SwervePath(
//                                new QuinticHermiteSpline(secondPickup, beforeSecondPickup),
//                                secondPickup.getHeading(), beforeSecondPickup.getHeading(),
//                                0, 2, 3, 5, 5,
//                                0, 0, 2.5, 0, 0.02, 0.5
//                        )
//                ),
//
//                new AutoLineDrive(4, 0.05,
//                        new SwervePath(
//                                new QuinticHermiteSpline(beforeSecondPickup, beforeAutoScore),
//                                beforeSecondPickup.getHeading(), beforeAutoScore.getHeading(),
//                                0, 1, 3, 5, 5,
//                                0, 0, 2.5, 0, 0.02, 0.5
//                        )
//                ),
//
//
//
//                new AutoScoreCommandGroup(scoring_third, StateMachine.RobotState.HIGH, StateMachine.PieceState.CUBE)
        );
    }
}
