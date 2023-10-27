package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ScoreIntake extends CommandBase {
    private Intake intake;

    public ScoreIntake(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        intake.setPosition(Constants.IntakeConstants.LOW_SCORE_POSITION);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return intake.getPositionError(Constants.IntakeConstants.LOW_SCORE_POSITION) < Constants.IntakeConstants.THRESHOLD;
    }
}
