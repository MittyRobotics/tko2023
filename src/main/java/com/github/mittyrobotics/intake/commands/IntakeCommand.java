package com.github.mittyrobotics.intake.commands;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.intake.IntakeConstants;
import com.github.mittyrobotics.intake.IntakeSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {
    public IntakeCommand() {
        setName("Intake Command");
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (StateMachine.getInstance().getCurrentRobotState() == StateMachine.RobotState.HP ||
                StateMachine.getInstance().getCurrentRobotState() == StateMachine.RobotState.GROUND)
            IntakeSubsystem.getInstance().setMotor(StateMachine.getInstance().shouldBeIntaking() ? IntakeConstants.INTAKE_SPEED : 0);

        else if (StateMachine.getInstance().getCurrentRobotState() == StateMachine.RobotState.MID ||
                StateMachine.getInstance().getCurrentRobotState() == StateMachine.RobotState.HIGH)
            IntakeSubsystem.getInstance().setMotor(StateMachine.getInstance().readyToShoot() &&
                    OI.getInstance().getOperatorController().getRightBumper() ? IntakeConstants.OUTTAKE_SPEED : 0);

        else if (StateMachine.getInstance().getCurrentRobotState() == StateMachine.RobotState.STOWED)
            IntakeSubsystem.getInstance().setMotor(0);

        else
            System.out.println("the fuck u manage to do dumb shit");
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
