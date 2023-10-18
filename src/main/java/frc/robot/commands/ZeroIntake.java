package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ZeroIntake extends CommandBase {
    private Intake intake;

    public ZeroIntake(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        intake.setMotor(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotor(0);
        intake.zeroIntake();
    }

    @Override
    public boolean isFinished() {
        return intake.getLimitSwitchTripped();
    }
}
