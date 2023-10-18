package frc.robot.commands.auto.routines;

import frc.robot.commands.PathFollowingCommand;
import frc.robot.commands.auto.AutoPathManager;
import frc.robot.commands.auto.AutoScoreHigh;
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
                low == null ? BALANCE_COMMUNITY_SIDE : low ? LOW_BALANCE_FAR_SIDE : HIGH_BALANCE_FAR_SIDE);
        addCommands(
                new AutoScoreHigh(conveyor, shooter, intake),
                new PathFollowingCommand(swerve, gyro, poseEstimator, balancePath)
        );
    }
}
