package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.autos.routines.AutoRoutine;
import frc.robot.commands.autos.routines.*;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;

public class AutoSelector {
    private Swerve swerve;
    private Gyro gyro;
    private PoseEstimator poseEstimator;

    private SendableChooser<Boolean> low = new SendableChooser<>();
    private SendableChooser<String> end = new SendableChooser<>();
    private SendableChooser<Integer> numberOfPieces = new SendableChooser<>();

    public AutoSelector(Swerve swerve, Gyro gyro, PoseEstimator poseEstimator) {
        this.swerve = swerve;
        this.gyro = gyro;
        this.poseEstimator = poseEstimator;

        low.setDefaultOption("Low", true);
        low.addOption("Middle", null);
        low.addOption("High (HP)", false);

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
                        return new Preload();
                    case "balance":
                        return new PreloadBalance(swerve, gyro, poseEstimator, low.getSelected());
                    case "taxi":
                        return new PreloadTaxi(swerve, gyro, poseEstimator, low.getSelected());
                    default:
                        return new Preload();
                }
            case 2:
                switch (end.getSelected()) {
                    case "nothing":
                        return new PreloadOne(swerve, gyro, poseEstimator, low.getSelected());
                    case "balance":
                        return new PreloadOneBalance(swerve, gyro, poseEstimator, low.getSelected());
                    default:
                        return new PreloadOne(swerve, gyro, poseEstimator, low.getSelected());
                }
            case 3:
                switch (end.getSelected()) {
                    case "nothing":
                        return new PreloadTwo(swerve, gyro, poseEstimator, low.getSelected());
                    case "balance":
                        return new PreloadTwoBalance(swerve, gyro, poseEstimator, low.getSelected());
                    default:
                        return new PreloadTwo(swerve, gyro, poseEstimator, low.getSelected());
                }
            default:
                return new DoNothing();
        }
    }
}
