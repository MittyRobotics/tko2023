package com.github.mittyrobotics.intake.commands;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.intake.IntakeConstants;
import com.github.mittyrobotics.intake.IntakeSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.Util;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoIntakeCommand extends CommandBase {
    boolean indexing = false;
    boolean outtaking = false;

    public AutoIntakeCommand() {
        setName("auto intake");
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (OI.getInstance().getOperatorController().getRightBumper()) {
            //Intake override
            IntakeSubsystem.getInstance().setMotor(IntakeConstants.OUTTAKE_SPEED);
            StateMachine.getInstance().setIntakeOff();
        } else if (OI.getInstance().getOperatorController().getLeftBumper()) {
            //Outtake override
            IntakeSubsystem.getInstance().setMotor(IntakeConstants.INTAKE_SPEED);
            StateMachine.getInstance().setIntakeStowing();
        } else if (StateMachine.getInstance().getIntakingState() == StateMachine.IntakeState.OUTTAKE) {
            //Outtake
            IntakeSubsystem.getInstance().setMotor(IntakeConstants.OUTTAKE_SPEED);
        } else if (StateMachine.getInstance().getIntakingState() == StateMachine.IntakeState.INTAKE) {
            //Intake
            IntakeSubsystem.getInstance().setMotor(IntakeConstants.INTAKE_SPEED);

            //If prox sensor detected index for another second then stow
            if (IntakeSubsystem.getInstance().proxSensorTrigger() && !indexing) {
                indexing = true;
                Util.triggerFunctionAfterTime(() -> {
                    ArmKinematics.setArmKinematics(new Angle(0), 0);
                    StateMachine.getInstance().setStateStowed();
                    StateMachine.getInstance().setIntakeStowing();
                    indexing = false;
                }, 1000);
            }
        } else if (StateMachine.getInstance().getIntakingState() == StateMachine.IntakeState.STOW) {
            //Piece stowed
            IntakeSubsystem.getInstance().setMotor(IntakeConstants.STOW_SPEED);
        } else if (StateMachine.getInstance().getIntakingState() == StateMachine.IntakeState.OFF) {
            //Intake off
            IntakeSubsystem.getInstance().setMotor(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
