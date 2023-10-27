package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RaiseIntake;
import frc.robot.commands.UnloadConveyor;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class AutoScoreHybrid extends SequentialCommandGroup {
    public AutoScoreHybrid(Conveyor conveyor, Intake intake) {
        super();

        addRequirements(conveyor, intake);

        addCommands(
                new RaiseIntake(intake),
                new UnloadConveyor(conveyor, () -> false).raceWith(new WaitCommand(2))
        );
    }
}
