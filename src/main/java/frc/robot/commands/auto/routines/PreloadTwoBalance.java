package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.BringCubeToHolding;
import frc.robot.commands.LowerIntake;
import frc.robot.commands.PathFollowingCommand;
import frc.robot.commands.RaiseIntake;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;
import frc.robot.util.autonomous.SwervePath;

import static frc.robot.commands.auto.AutoPathManager.PathName.*;

public class PreloadTwoBalance extends AutoRoutine {
    public PreloadTwoBalance(AutoPathManager pathManager,
                             Swerve swerve, Gyro gyro, PoseEstimator poseEstimator,
                             Conveyor conveyor, Shooter shooter, Intake intake,
                             Boolean low) {
        super();

        SwervePath firstPiecePath = pathManager.paths.get(
                low == null ? null : low ? LOW_TO_FIRST_PIECE : HIGH_TO_FIRST_PIECE);
        SwervePath firstScorePath = pathManager.paths.get(
                low == null ? null : low ? LOW_FIRST_PIECE_TO_SCORE : HIGH_FIRST_PIECE_TO_SCORE);
        SwervePath secondPiecePath = pathManager.paths.get(
                low == null ? null : low ? LOW_TO_SECOND_PIECE : HIGH_TO_SECOND_PIECE);
        SwervePath secondScorePath = pathManager.paths.get(
                low == null ? null : low ? LOW_SECOND_PIECE_TO_SCORE : HIGH_SECOND_PIECE_TO_SCORE);
        SwervePath balancePath = pathManager.paths.get(
                low == null ? null : low ? LOW_BALANCE_FAR_SIDE : HIGH_BALANCE_FAR_SIDE);
        addCommands(
                new AutoScoreHigh(conveyor, shooter),
                new PathFollowingCommand(swerve, gyro, poseEstimator, firstPiecePath),
                new LowerIntake(intake),
                new ParallelCommandGroup(
                        new PathFollowingCommand(swerve, gyro, poseEstimator, low == null ? null : pathManager, 10),
                        new BringCubeToHolding(conveyor)
                ),
                new RaiseIntake(intake),
                new PathFollowingCommand(swerve, gyro, poseEstimator, firstScorePath),
                new AutoScoreMid(conveyor, shooter),
                new PathFollowingCommand(swerve, gyro, poseEstimator, secondPiecePath),
                new LowerIntake(intake),
                new ParallelCommandGroup(
                        new PathFollowingCommand(swerve, gyro, poseEstimator, low == null ? null : pathManager, 10),
                        new BringCubeToHolding(conveyor)
                ),
                new RaiseIntake(intake),
                new PathFollowingCommand(swerve, gyro, poseEstimator, secondScorePath),
                new AutoScoreHybrid(conveyor, intake),
                new PathFollowingCommand(swerve, gyro, poseEstimator, balancePath),
                new AutoBalance(swerve, gyro, poseEstimator, low == null)
        );
    }
}
