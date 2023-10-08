package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class BringCubeToHolding extends CommandBase {
    private Conveyor conveyor;

    private double initTime;

    public BringCubeToHolding(Conveyor conveyor) {
        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
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
        return conveyor.getLimitSwitchTripped() || Timer.getFPGATimestamp() - initTime > 5;
    }
}
