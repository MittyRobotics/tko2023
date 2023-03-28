package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.SwervePath;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.pivot.ArmKinematics;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPickupCommand extends SequentialCommandGroup {

    public AutoPickupCommand(int index, double maxvel, double maxaccel, double maxdecel, double startvel, double endvel, boolean auto) {
        super();

        QuinticHermiteSpline spline = ArmKinematics.getSplineToGamePiece();

        addCommands(
                new InstantCommand(() ->         LoggerInterface.getInstance().put("AUTO PICKUP", true)
),
                new InstantCommand(() -> LedSubsystem.getInstance().setAltColor(LedSubsystem.Color.BLUE)),
                new AutoDrivePickupCommand(2, 0.05, index,
                    new SwervePath(spline, 6, maxvel, maxaccel, maxdecel, startvel, endvel, auto),
                        SwerveConstants.MAX_ANGULAR_VEL / 3
                ),
                new InstantCommand(() -> LedSubsystem.getInstance().disableDriveAltColor())
        );
    }
}
