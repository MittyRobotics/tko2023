package frc.robot.commands.auto.routines;

import frc.robot.commands.auto.PathFollowingCommand;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoPathManager;
import frc.robot.subsystems.*;
import frc.robot.util.autonomous.SwervePath;

import static frc.robot.commands.auto.AutoPathManager.PathName.*;

public class PreloadBalance extends AutoRoutine {
    public PreloadBalance(AutoPathManager pathManager,
                          Swerve swerve, Gyro gyro, PoseEstimator poseEstimator,
                          Conveyor conveyor, Shooter shooter, Intake intake,
                          Boolean low) {
        super();

        SwervePath balancePath = pathManager.paths.get(
                low == null ? null : low ? LOW_BALANCE_FAR_SIDE : HIGH_BALANCE_FAR_SIDE);
        addCommands(
                new Preload(conveyor, shooter),
                new PathFollowingCommand(swerve, gyro, poseEstimator, balancePath),
                new AutoBalance(swerve, gyro, poseEstimator, low == null)
        );
    }
}
