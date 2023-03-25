package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.pivot.ArmKinematics;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.logging.Logger;

public class AutoPickupCommand extends SequentialCommandGroup {

    public AutoPickupCommand(int index, boolean auto) {
        super();
//        if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.NONE) return;
//        boolean isCone = StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CONE;
        Pose init = Odometry.getInstance().getState();
//        double angle = ArmKinematics.getAngleToGamePiece(isCone, index);
//        if (Double.isNaN(angle)) return;

        double angle = ArmKinematics.getLastAngleToGamePiece();

        Pose end = new Pose(Point.add(init.getPosition(), new Point(1000 * Math.cos(angle), 1000 * Math.sin(angle))), init.getHeading());
        addCommands(
                new InstantCommand(() ->         LoggerInterface.getInstance().put("AUTO PICKUP", true)
),
                new InstantCommand(() -> LedSubsystem.getInstance().setAltColor(LedSubsystem.Color.BLUE)),
                new AutoDrivePickupCommand(2, 0.05, index,
                        new OldSwervePath(new QuinticHermiteSpline(init, end), init.getHeading(), new Angle(angle),
                                0, 0, 3, 6, 6,
                                0, 0, 5, 0, 0.02, 0.5
                        ), auto
                ),
                new InstantCommand(() -> LedSubsystem.getInstance().disableDriveAltColor())
        );
    }
}
