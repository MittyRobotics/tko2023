package frc.robot.commands.auto.routines;

import frc.robot.commands.auto.PathFollowingCommand;
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

        SwervePath balancePath = pathManager.paths.get(
                low == null ? null : low ? LOW_BALANCE_FAR_SIDE : HIGH_BALANCE_FAR_SIDE);
        addCommands(
                new PreloadTwo(pathManager,
                        swerve, gyro, poseEstimator,
                        conveyor, shooter, intake,
                        low),
                new PathFollowingCommand(swerve, gyro, poseEstimator, balancePath),
                new AutoBalance(swerve, gyro, poseEstimator, low == null)
        );
    }
}
