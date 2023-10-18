package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class UnloadConveyor extends CommandBase {
    private Conveyor conveyor;

    public UnloadConveyor(Conveyor conveyor) {
        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        conveyor.setMotor(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
