package frc.robot.commands.auto.routines;

import frc.robot.commands.auto.AutoScoreHigh;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Preload extends AutoRoutine {
    public Preload(Conveyor conveyor, Shooter shooter, Intake intake) {
        super();

        addCommands(
                new AutoScoreHigh(conveyor, shooter)
        );
    }
}
