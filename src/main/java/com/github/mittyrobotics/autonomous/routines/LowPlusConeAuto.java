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

public class LowPlusConeAuto extends SequentialCommandGroup {
    public LowPlusConeAuto(boolean leftSide, boolean balance) {
        super();

        Pose scoring = Odometry.getInstance().getScoringZone(leftSide ? 8 : 1)[2];
        Pose scoring_second = Odometry.getInstance().getScoringZone(leftSide ? 8 : 1)[1];

        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                new Angle(leftSide ? 0 : Math.PI));

        Pose firstCone = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 195 : -195,5)),
                starting.getHeading());

        Pose beforeFirstCone = new Pose(Point.add(firstCone.getPosition(), new Point(leftSide ? -80 : 80, 0)),
                starting.getHeading());

        Pose beforeAutoScore = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 30 : -30, 0)),
                starting.getHeading());

        addCommands(
                // FIRST CONE
                new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                        Odometry.getInstance().FIELD_LEFT_SIDE ? 3 : 0 //left vs right BACK cam
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
                new InstantCommand(() -> StateMachine.getInstance().setStateCube()),
                new InstantCommand(() -> OI.getInstance().handleGround()),

                new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(beforeFirstCone, firstCone),
                                10, 2, 2, 2, 2, 0, true
                        ), leftSide ? 0 : Math.PI, 2, 0.05,
                        0, 1, 3, 0, 0.02, true
                ),


                new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                        Odometry.getInstance().FIELD_LEFT_SIDE ? 2 : 1 //left vs right FRONT cam
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
                                10, 2, 2, 2, 0, 0, true
                        ), leftSide ? Math.PI : 0, 2, 1,
                        0.2, 0.8, 3, 0, 0.02, true
                ),
//
//
                new AutoScoreCommandGroup(leftSide ? 8 : 1, 1, StateMachine.RobotState.HIGH, StateMachine.PieceState.CUBE)
        );
    }
}
