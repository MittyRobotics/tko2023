package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class OuttakeConveyor extends CommandBase {
    Conveyor conveyor;

    public OuttakeConveyor(Conveyor conveyor) {
        super();

        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        conveyor.setMotor(-0.2);
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
