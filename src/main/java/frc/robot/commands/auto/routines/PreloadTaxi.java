package frc.robot.commands.auto.routines;

import frc.robot.commands.PathFollowingCommand;
import frc.robot.commands.auto.AutoPathManager;
import frc.robot.commands.auto.AutoScoreHigh;
import frc.robot.subsystems.*;
import frc.robot.util.autonomous.SwervePath;

import static frc.robot.commands.auto.AutoPathManager.PathName.*;

public class PreloadTaxi extends AutoRoutine {
    public PreloadTaxi(AutoPathManager pathManager,
                       Swerve swerve, Gyro gyro, PoseEstimator poseEstimator,
                       Conveyor conveyor, Shooter shooter, Intake intake,
                       Boolean low) {
        super();

        SwervePath taxiPath = pathManager.paths.get(
                low == null ? null : low ? LOW_TAXI : HIGH_TAXI);
        addCommands(
                new AutoScoreHigh(conveyor, shooter),
                new PathFollowingCommand(swerve, gyro, poseEstimator, taxiPath)
        );
    }
}
