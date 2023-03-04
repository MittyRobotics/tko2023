package com.github.mittyrobotics.intake.commands;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.intake.IntakeConstants;
import com.github.mittyrobotics.intake.IntakeSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoIntakeCommand extends CommandBase {
    boolean indexing = false;

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (OI.getInstance().getOperatorController().getRightBumper()) {
            IntakeSubsystem.getInstance().setMotor(IntakeConstants.OUTTAKE_SPEED);
            triggerFunctionAfterTime(() -> {
                ArmKinematics.setArmKinematics(new Angle(0), 0);
                StateMachine.getInstance().setStateStowed();
            }, 1000);
        } else if (StateMachine.getInstance().getIntaking()) {
            IntakeSubsystem.getInstance().setMotor(IntakeConstants.INTAKE_SPEED);
            if (IntakeSubsystem.getInstance().proxSensorTrigger() && !indexing) {
                indexing = true;
                triggerFunctionAfterTime(() -> {
                    StateMachine.getInstance().setIntaking(false);
                    indexing = false;
                }, 1000);
                triggerFunctionAfterTime(() -> {
                    ArmKinematics.setArmKinematics(new Angle(0), 0);
                    StateMachine.getInstance().setStateStowed();
                }, 2000);
            }
        } else {
            IntakeSubsystem.getInstance().setMotor(-0.1);
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

    private void triggerFunctionAfterTime(Runnable runnable, long time){
        new java.util.Timer().schedule(
                new java.util.TimerTask() {
                    @Override
                    public void run() {
                        runnable.run();
                    }
                },
                time
        );
    }
}
