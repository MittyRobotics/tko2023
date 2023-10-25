package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

import java.util.function.BooleanSupplier;

public class UnloadConveyor extends CommandBase {
    private Conveyor conveyor;
    private BooleanSupplier dir;

    public UnloadConveyor(Conveyor conveyor, BooleanSupplier dir) {
        this.conveyor = conveyor;

        this.dir = dir;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        conveyor.setMotor(dir.getAsBoolean() ? 0.5 : -0.5);
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
