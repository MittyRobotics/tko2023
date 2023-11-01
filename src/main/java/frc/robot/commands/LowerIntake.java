package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class LowerIntake extends CommandBase {
    private Intake intake;

    public LowerIntake(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        intake.setPosition(Constants.IntakeConstants.DOWN_POSITION);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
