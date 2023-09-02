package com.github.mittyrobotics.intake.commands;

import com.github.mittyrobotics.arm.StateMachine;
import com.github.mittyrobotics.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.github.mittyrobotics.intake.IntakeConstants.*;

public class IntakeDefaultCommand extends CommandBase {
    StateMachine.IntakeState intakeState;

    public IntakeDefaultCommand() {
        setName("Intake Default Command");
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        intakeState = StateMachine.getIntakeState();

        switch (intakeState) {
            case EMPTY:
                IntakeSubsystem.getInstance().setMotor(EMPTY_SPEED);
                break;
            case STOWING:
                IntakeSubsystem.getInstance().setMotor(STOWING_SPEED);
                break;
            case INTAKING:
                IntakeSubsystem.getInstance().setMotor(INTAKING_SPEED);
                break;
            case OUTTAKING:
                if (StateMachine.getPieceState() == StateMachine.PieceState.CUBE)
                    IntakeSubsystem.getInstance().setMotor(CUBE_SCORE_SPEED);
                else IntakeSubsystem.getInstance().setMotor(OUTTAKING_SPEED);
                break;
            default:
                IntakeSubsystem.getInstance().setMotor(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
