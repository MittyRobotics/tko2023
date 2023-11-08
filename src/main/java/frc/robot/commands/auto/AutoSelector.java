package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.auto.routines.AutoRoutine;
import frc.robot.commands.auto.routines.*;
import frc.robot.subsystems.*;

public class AutoSelector {
    private AutoPathManager pathManager;

    private Swerve swerve;
    private Gyro gyro;
    private PoseEstimator poseEstimator;

    private Conveyor conveyor;
    private Shooter shooter;
    private Intake intake;

    private SendableChooser<Boolean> start = new SendableChooser<>();
    private SendableChooser<String> end = new SendableChooser<>();
    private SendableChooser<Integer> numberOfPieces = new SendableChooser<>();

    public AutoSelector(Swerve swerve, Gyro gyro, PoseEstimator poseEstimator,
                        Conveyor conveyor, Shooter shooter, Intake intake) {
        this.pathManager = new AutoPathManager(poseEstimator, swerve, gyro);

        this.swerve = swerve;
        this.gyro = gyro;
        this.poseEstimator = poseEstimator;

        this.conveyor = conveyor;
        this.shooter = shooter;
        this.intake = intake;

        start.setDefaultOption("Low", true);
        start.addOption("Middle", null);
        start.addOption("High (HP)", false);

        end.setDefaultOption("Nothing", "nothing");
        end.addOption("Balance (Only for 1 and 2 piece)", "balance");
        end.addOption("Taxi (Only for non-middle placement)", "taxi");

        numberOfPieces.setDefaultOption("Zero", 0);
        numberOfPieces.addOption("One", 1);
        numberOfPieces.addOption("Two", 2);
        numberOfPieces.addOption("Three", 3);
    }

    public AutoRoutine getAuto() {
        switch (numberOfPieces.getSelected()) {
            case 0:
                return new DoNothing();
            case 1:
                switch (end.getSelected()) {
                    case "nothing":
                        return new Preload(conveyor, shooter, intake);
                    case "balance":
                        return new PreloadBalance(pathManager,
                                swerve, gyro, poseEstimator,
                                conveyor, shooter, intake,
                                start.getSelected());
                    case "taxi":
                        return new PreloadTaxi(pathManager,
                                swerve, gyro, poseEstimator,
                                conveyor, shooter, intake,
                                start.getSelected());
                    default:
                        return new Preload(conveyor, shooter, intake);
                }
            case 2:
                switch (end.getSelected()) {
                    case "nothing":
                        return new PreloadOne(pathManager,
                                swerve, gyro, poseEstimator,
                                conveyor, shooter, intake,
                                start.getSelected());
                    case "balance":
                        return new PreloadOneBalance(pathManager,
                                swerve, gyro, poseEstimator,
                                conveyor, shooter, intake,
                                start.getSelected());
                    case "taxi":
                        return new PreloadOneTaxi(pathManager,
                                swerve, gyro, poseEstimator,
                                conveyor, shooter, intake,
                                start.getSelected());
                    default:
                        return new PreloadOne(pathManager,
                                swerve, gyro, poseEstimator,
                                conveyor, shooter, intake,
                                start.getSelected());
                }
            case 3:
                switch (end.getSelected()) {
                    case "nothing":
                        return new PreloadTwo(pathManager,
                                swerve, gyro, poseEstimator,
                                conveyor, shooter, intake,
                                start.getSelected());
                    case "balance":
                        return new PreloadTwoBalance(pathManager,
                                swerve, gyro, poseEstimator,
                                conveyor, shooter, intake,
                                start.getSelected());
                    case "taxi":
                        return new PreloadTwoTaxi(pathManager,
                                swerve, gyro, poseEstimator,
                                conveyor, shooter, intake,
                                start.getSelected());
                    default:
                        return new PreloadTwo(pathManager,
                                swerve, gyro, poseEstimator,
                                conveyor, shooter, intake,
                                start.getSelected());
                }
            default:
                return new DoNothing();
        }
    }
}
