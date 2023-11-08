package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class ZeroIntake extends CommandBase {
    private Intake intake;
    private Conveyor conveyor;

    public ZeroIntake(Intake intake, Conveyor conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        intake.setMotor(0.2);
        conveyor.setMotor(0);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotor(0);
        intake.zeroIntake();
    }

    @Override
    public boolean isFinished() {
        return intake.getLimitSwitchTripped() || intake.hasBeenZeroed();
    }
}
