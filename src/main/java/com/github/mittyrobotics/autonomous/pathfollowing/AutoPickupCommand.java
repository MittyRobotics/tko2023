package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.pivot.ArmKinematics;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPickupCommand extends SequentialCommandGroup {

    public AutoPickupCommand(boolean isCone, int index, boolean auto) {
        super();
        Pose init = Odometry.getInstance().getState();
        double angle = ArmKinematics.getAngleToGamePiece(isCone, index);
        if (Double.isNaN(angle)) return;

        Pose end = new Pose(Point.add(init.getPosition(), new Point(1000 * Math.cos(angle), 1000 * Math.sin(angle))), init.getHeading());
        addCommands(
                new InstantCommand(() -> LedSubsystem.getInstance().setAltColor(LedSubsystem.Color.BLUE)),
                new AutoDrivePickupCommand(2, 0.05, isCone, index,
                        new SwervePath(new QuinticHermiteSpline(init, end), init.getHeading(), new Angle(angle),
                                0, 0, 3, 6, 6,
                                0, 0, 0.0, 0, 0.00, 0.5
                        ), auto
                ),
                new InstantCommand(() -> LedSubsystem.getInstance().disableDriveAltColor())
        );
    }
}
