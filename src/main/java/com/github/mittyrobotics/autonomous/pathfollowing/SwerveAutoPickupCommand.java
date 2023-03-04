package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.pivot.ArmKinematics;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.json.JSONException;

public class SwerveAutoPickupCommand extends SequentialCommandGroup {

    public SwerveAutoPickupCommand(boolean isCone, int index) {
        super();
        Pose init = Odometry.getInstance().getState();
        double[] vectorToGamePiece;
        try {
            vectorToGamePiece = ArmKinematics.getVectorToGamePiece(isCone, index);
        } catch (Exception e) {
            vectorToGamePiece = new double[]{0, 0, 0};
        }
        Pose end = new Pose(Point.add(init.getPosition(), new Point(vectorToGamePiece[0], vectorToGamePiece[1])), Odometry.getInstance().getState().getHeading());
        addCommands(new SwerveAutoDriveToTargetCommand(0.05, 0.05,
                new SwervePath(new QuinticHermiteSpline(init, end),
                        SwerveSubsystem.getInstance().getPose().getHeading(), SwerveSubsystem.getInstance().getPose().getHeading(),
                        0, 0, 3., 12., 3, 0.0, 0.2, 0.0, 0, 0.00, 0.5)));
    }
}
