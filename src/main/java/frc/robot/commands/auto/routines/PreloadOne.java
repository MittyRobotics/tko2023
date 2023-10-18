package frc.robot.commands.auto.routines;

import frc.robot.commands.PathFollowingCommand;
import frc.robot.commands.auto.AutoPathManager;
import frc.robot.commands.auto.AutoScoreHigh;
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
        addCommands(
                new AutoScoreHigh(conveyor, shooter, intake),
                new PathFollowingCommand(swerve, gyro, poseEstimator, firstPiecePath)
        );
    }
}
