package frc.robot.commands.auto.routines;

import frc.robot.commands.auto.PathFollowingCommand;
import frc.robot.commands.auto.AutoPathManager;
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

        poseEstimator.setState(taxiPath.getByT(0).getPoint().getX(), taxiPath.getByT(0).getPoint().getY(),
                taxiPath.getByT(0).getHeading().getRadians());

        addCommands(
                new Preload(conveyor, shooter, intake),
                new PathFollowingCommand(swerve, gyro, poseEstimator, taxiPath)
        );
    }
}
