package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.routines.AutoRoutine;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    private Swerve swerve;
    private Gyro gyro;
    private PoseEstimator poseEstimator;
    private boolean communitySide;

    public AutoBalance(Swerve swerve, Gyro gyro, PoseEstimator poseEstimator, boolean communitySide) {
        super();

        this.swerve = swerve;
        this.gyro = gyro;
        this.poseEstimator = poseEstimator;
        this.communitySide = communitySide;

        addRequirements(poseEstimator, swerve);
    }
}
