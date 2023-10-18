package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.HighFlywheel;
import frc.robot.commands.UnloadConveyor;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.*;

public class AutoScoreHigh extends SequentialCommandGroup {
    public AutoScoreHigh(Conveyor conveyor, Shooter shooter, Intake intake) {
        super();

        addRequirements(conveyor, shooter, intake);

        SequentialCommandGroup unloadSequence = new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.getVelocityError() < ShooterConstants.THRESHOLD),
                new UnloadConveyor(conveyor).raceWith(new WaitCommand(1))
        );
        addCommands(
                new ParallelCommandGroup(
                        new HighFlywheel(shooter),
                        unloadSequence
                ).raceWith(new WaitCommand(2))
        );
    }
}
