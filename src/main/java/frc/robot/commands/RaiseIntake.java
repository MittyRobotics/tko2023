package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RaiseIntake extends CommandBase {
    private Intake intake;

    public RaiseIntake(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        intake.setPosition(Constants.IntakeConstants.UP_POSITION);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return intake.getPositionError(Constants.IntakeConstants.UP_POSITION) < Constants.IntakeConstants.THRESHOLD;
    }
}
