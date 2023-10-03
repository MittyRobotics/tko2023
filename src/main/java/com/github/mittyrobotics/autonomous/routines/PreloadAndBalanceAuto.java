package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PreloadAndBalanceAuto extends SequentialCommandGroup {
    public PreloadAndBalanceAuto(boolean leftSide, boolean bal, boolean taxi, boolean low) {
        super();

        //tag should be 1, 2, 3 from bottom to top
        com.github.mittyrobotics.util.math.Pose p = Odometry.getInstance().getScoringZone(leftSide ? 7 : 2)[2];
        Pose scoring = new Pose(new Point(p.getPoint().getX(), p.getPoint().getY()), new Angle(p.getAngle().getRadians()));
        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                scoring.getHeading());

        if (bal) {
            addCommands(
                    new InstantCommand(() -> SmartDashboard.putString("Test", "running")),
                    new InitAutoCommand(starting),
                    new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),
                    new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),

                    new WaitCommand(2),

                    new Balance(false)
            );
        } else if (taxi) {
            addCommands(
                new PTaxi(low, Odometry.getInstance().FIELD_LEFT_SIDE, StateMachine.PieceState.CONE)
            );
        } else {
            addCommands(
                    new InitAutoCommand(starting),
                    new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),
                    new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE)
            );
        }

    }
}
