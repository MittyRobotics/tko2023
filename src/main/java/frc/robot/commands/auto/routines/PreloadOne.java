package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.LowerIntake;
import frc.robot.commands.auto.PathFollowingCommand;
import frc.robot.commands.RaiseIntake;
import frc.robot.commands.auto.AutoPathManager;
import frc.robot.commands.auto.AutoScoreMid;
import frc.robot.subsystems.*;
import frc.robot.util.autonomous.SwervePath;

import static frc.robot.commands.auto.AutoPathManager.PathName.*;

public class PreloadOne extends AutoRoutine {
    public PreloadOne(AutoPathManager pathManager,
                      Swerve swerve, Gyro gyro, PoseEstimator poseEstimator,
                      Conveyor conveyor, Shooter shooter, Intake intake,
                      Boolean low) {
        super();

        SwervePath firstPiecePath = pathManager.paths.get(
                low == null ? null : low ? LOW_TO_FIRST_PIECE : HIGH_TO_FIRST_PIECE);
        SwervePath firstScorePath = pathManager.paths.get(
                low == null ? null : low ? LOW_FIRST_PIECE_TO_SCORE : HIGH_FIRST_PIECE_TO_SCORE);

        poseEstimator.setState(firstPiecePath.getByT(0).getPoint().getX(), firstPiecePath.getByT(0).getPoint().getY(),
                firstPiecePath.getByT(0).getHeading().getRadians());

        addCommands(
                new Preload(conveyor, shooter, intake),
                new PathFollowingCommand(swerve, gyro, poseEstimator, firstPiecePath),
                new PathFollowingCommand(swerve, gyro, poseEstimator, low == null ? null : () -> pathManager.getGroundIntakingPath(10))
                        .alongWith(new IntakeCube(conveyor)
                        .raceWith(new LowerIntake(intake))),
                new PathFollowingCommand(swerve, gyro, poseEstimator, firstScorePath),
                new AutoScoreMid(conveyor, shooter)
        );
    }
}
