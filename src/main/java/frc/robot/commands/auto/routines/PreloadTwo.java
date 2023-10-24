package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.BringCubeToHolding;
import frc.robot.commands.LowerIntake;
import frc.robot.commands.auto.PathFollowingCommand;
import frc.robot.commands.RaiseIntake;
import frc.robot.commands.auto.AutoPathManager;
import frc.robot.commands.auto.AutoScoreHybrid;
import frc.robot.subsystems.*;
import frc.robot.util.autonomous.SwervePath;

import static frc.robot.commands.auto.AutoPathManager.PathName.*;

public class PreloadTwo extends AutoRoutine {
    public PreloadTwo(AutoPathManager pathManager,
                      Swerve swerve, Gyro gyro, PoseEstimator poseEstimator,
                      Conveyor conveyor, Shooter shooter, Intake intake,
                      Boolean low) {
        super();

        SwervePath secondPiecePath = pathManager.paths.get(
                low == null ? null : low ? LOW_TO_SECOND_PIECE : HIGH_TO_SECOND_PIECE);
        SwervePath secondScorePath = pathManager.paths.get(
                low == null ? null : low ? LOW_SECOND_PIECE_TO_SCORE : HIGH_SECOND_PIECE_TO_SCORE);
        addCommands(
                new PreloadOne(pathManager,
                        swerve, gyro, poseEstimator,
                        conveyor, shooter, intake,
                        low),
                new PathFollowingCommand(swerve, gyro, poseEstimator, secondPiecePath),
                new LowerIntake(intake),
                new ParallelCommandGroup(
                        new PathFollowingCommand(swerve, gyro, poseEstimator, low == null ? null : () -> pathManager.getGroundIntakingPath(10)),
                        new BringCubeToHolding(conveyor)
                ),
                new RaiseIntake(intake),
                new PathFollowingCommand(swerve, gyro, poseEstimator, secondScorePath),
                new AutoScoreHybrid(conveyor, intake)
        );
    }
}
