package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.arm.AutoScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.SwerveAutoPickupCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePurePursuitCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OneConePlusPreload extends SequentialCommandGroup {
    Pose init = Odometry.getInstance().getState();
    Pose conePPEndpoint = new Pose(new Point(0, 0), new Angle(Odometry.getInstance().FIELD_LEFT_SIDE ? 0 : Math.PI));

    public OneConePlusPreload() {
        super();

        addCommands(
                new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE, true),
//                new SwervePurePursuitCommand(2, 0.05, ),
                new InstantCommand(OI.getInstance()::handleGround),
//                new SwerveAutoPickupCommand(true, 0),
                new WaitCommand(1),
                new InstantCommand(OI.getInstance()::zeroAll)
//                new SwervePurePursuitCommand(2, 0.05, ),
//                new AutoScoreCommand(Odometry.getInstance().getClosestScoringZone()[0], StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE, true)
        );
    }
}
