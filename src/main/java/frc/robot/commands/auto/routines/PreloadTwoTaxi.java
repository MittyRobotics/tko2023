package frc.robot.commands.auto.routines;

import frc.robot.commands.auto.PathFollowingCommand;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;
import frc.robot.util.autonomous.SwervePath;

import static frc.robot.commands.auto.AutoPathManager.PathName.*;

public class PreloadTwoTaxi extends AutoRoutine {
    public PreloadTwoTaxi(AutoPathManager pathManager,
                             Swerve swerve, Gyro gyro, PoseEstimator poseEstimator,
                             Conveyor conveyor, Shooter shooter, Intake intake,
                             Boolean low) {
        super();

        SwervePath taxiPath = pathManager.paths.get(
                low == null ? null : low ? LOW_TAXI : HIGH_TAXI);
        addCommands(
                new PreloadTwo(pathManager,
                        swerve, gyro, poseEstimator,
                        conveyor, shooter, intake,
                        low),
                new PathFollowingCommand(swerve, gyro, poseEstimator, taxiPath)
        );
    }
}
