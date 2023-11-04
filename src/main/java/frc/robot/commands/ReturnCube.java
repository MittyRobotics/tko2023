package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ReturnCube extends CommandBase {
    private Conveyor conveyor;
    private double initPos;

    public ReturnCube(Conveyor conveyor) {
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        initPos = conveyor.getPosition();
        System.out.println("\n\n\n started \n\n\n");
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
        return initPos - conveyor.getPosition() > 3;
    }
}
