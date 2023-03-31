package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.arm.AutoScoreCommandGroup;
import com.github.mittyrobotics.autonomous.pathfollowing.AutoLineDrive;
import com.github.mittyrobotics.autonomous.pathfollowing.OldSwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.PathFollowingCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.SwervePath;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PrePlusTwoAuto extends SequentialCommandGroup {
    public PrePlusTwoAuto(boolean low, boolean leftSide, boolean balance) {
        super();

        int tag_id = leftSide ? (low ? 8 : 6) : (low ? 1 : 3);
        int second_index = 1;

        Pose scoring = Odometry.getInstance().getScoringZone(tag_id)[low ? 2 : 0];

        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                new Angle(leftSide ? 0 : Math.PI));

        Pose firstCone = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 195 : -195, low ? 5 : -5)),
                starting.getHeading());

        Pose beforeFirstCone = new Pose(Point.add(firstCone.getPosition(), new Point(leftSide ? -100 : 100, 0)),
                starting.getHeading());

        Pose beforeAutoScore = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 18 : -18, 0)),
                starting.getHeading());

        Point starting_second = Point.add(
                Odometry.getInstance().getScoringZone(tag_id)[second_index].getPosition(),
                new Point(leftSide ? 32 : -32, 0));

        Point beforeBalance = Point.add(starting_second,
                new Point(leftSide ? 12 : -12, (low ? 1 : -1) *
                        (second_index == 1 ? 44 : 22)));



        if(!balance) {
            //NO BALANCE PRELOAD + ONE
            addCommands(
                    // FIRST CONE
                    new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                            Odometry.getInstance().FIELD_LEFT_SIDE ? (low ? 3 : 0) : (low ? 0 : 3) //left vs right BACK cam
                    )),

                    new InitAutoCommand(new Pose(starting.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                    new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),

                    new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),


                    new PathFollowingCommand(
                            new SwervePath(
                                    new QuinticHermiteSpline(starting, beforeFirstCone),
                                    10, 2, 2, 2, 0, 2, true
                            ), leftSide ? 0 : Math.PI, 4, 1,
                            0.2, 0.8, 3, 0, 0.02, true
                    ),

                    // INTAKE
                    new InstantCommand(() -> StateMachine.getInstance().setState(piece)),
                    new InstantCommand(() -> OI.getInstance().handleGround()),

                    new PathFollowingCommand(
                            new SwervePath(
                                    new QuinticHermiteSpline(beforeFirstCone, firstCone),
                                    10, 2, 2, 2, 2, 0, true
                            ), leftSide ? 0 : Math.PI, 2, 0.05,
                            0, 1, 3, 0, 0.02, true
                    ),


                    new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                            Odometry.getInstance().FIELD_LEFT_SIDE ? (low ? 2 : 1) : (low ? 1 : 2) //left vs right FRONT cam
                    )),
                    new InstantCommand(() -> {
                        OI.getInstance().zeroAll();
                        StateMachine.getInstance().setIntakeStowing();
                        Odometry.getInstance().setScoringCam(true);
                    }),

                    new PathFollowingCommand(
                            new SwervePath(
                                    new QuinticHermiteSpline(
                                            new Pose(firstCone.getPosition(), new Angle(leftSide ? Math.PI : 0)),
                                            new Pose(beforeAutoScore.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                                    10, 2, 2, 2, 0, 1, true
                            ), leftSide ? Math.PI : 0, 2, 1,
                            0.2, 0.8, 3, 0, 0.02, true
                    ),

                    new AutoScoreCommandGroup(tag_id, second_index, StateMachine.RobotState.HIGH, piece,
                            1, 1.5, 1.5, 1, 0)
            );
        } else {
            addCommands(
                    // FIRST CONE
                    new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                            Odometry.getInstance().FIELD_LEFT_SIDE ? (low ? 3 : 0) : (low ? 0 : 3) //left vs right BACK cam
                    )),

                    new InitAutoCommand(new Pose(starting.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                    new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),

                    new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),


                    new PathFollowingCommand(
                            new SwervePath(
                                    new QuinticHermiteSpline(starting, beforeFirstCone),
                                    10, 3, 5, 5, 0, 2.5, true
                            ), leftSide ? 0 : Math.PI, 6, 3,
                            0.1, 0.6, 3, 0, 0.01, true
                    ),

                    // INTAKE
                    new InstantCommand(() -> StateMachine.getInstance().setState(piece)),
                    new InstantCommand(() -> OI.getInstance().handleGround()),

                    new PathFollowingCommand(
                            new SwervePath(
                                    new QuinticHermiteSpline(beforeFirstCone, firstCone),
                                    10, 2.5, 5, 2, 2.5, 0, true
                            ), leftSide ? 0 : Math.PI, 2, 0.05,
                            0, 1, 3, 0, 0.02, true
                    ),


                    new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                            Odometry.getInstance().FIELD_LEFT_SIDE ? (low ? 2 : 1) : (low ? 1 : 2) //left vs right FRONT cam
                    )),
                    new InstantCommand(() -> {
                        OI.getInstance().zeroAll();
                        StateMachine.getInstance().setIntakeStowing();
                        Odometry.getInstance().setScoringCam(true);
                    }),

                    new PathFollowingCommand(
                            new SwervePath(
                                    new QuinticHermiteSpline(
                                            new Pose(firstCone.getPosition(), new Angle(leftSide ? Math.PI : 0)),
                                            new Pose(beforeAutoScore.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                                    10, 4, 5, 2, 0, 1, true
                            ), leftSide ? Math.PI : 0, 5, 1,
                            0.2, 0.8, 3, 0, 0.02, true
                    ),
//
//
                    new AutoScoreCommandGroup(tag_id, second_index, StateMachine.RobotState.HIGH, piece,
                            1, 1.5, 1.5, 1, 0),


                    new PathFollowingCommand(
                            new SwervePath(
                                    new QuinticHermiteSpline(starting_second, beforeBalance),
                                    5, 3, 3, 3, 0, 3, true
                            ), leftSide ? Math.PI : 0, 4, 1,
                            0, 1, 3, 0, 0.02, true
                    )

//                    , new Balance(false)
            );
        }
    }
}
