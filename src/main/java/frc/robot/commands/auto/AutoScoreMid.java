package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.MidFlywheel;
import frc.robot.commands.UnloadConveyor;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.ShooterConstants;

public class AutoScoreMid extends SequentialCommandGroup {
    public AutoScoreMid(Conveyor conveyor, Shooter shooter, Intake intake) {
        super();

        addRequirements(conveyor, shooter, intake);

        SequentialCommandGroup unloadSequence = new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.getVelocityError() < ShooterConstants.THRESHOLD),
                new UnloadConveyor(conveyor).raceWith(new WaitCommand(1))
        );
        addCommands(
                new ParallelCommandGroup(
                        new MidFlywheel(shooter),
                        unloadSequence
                ).raceWith(new WaitCommand(2))
        );
    }
}
