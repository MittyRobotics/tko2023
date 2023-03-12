package com.github.mittyrobotics.intake.commands;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.intake.IntakeConstants;
import com.github.mittyrobotics.intake.IntakeSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.Util;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.PriorityQueue;

public class AutoIntakeCommand extends CommandBase {
    boolean indexing = false;
    boolean outtaking = false;

    private final double threshold = 15;

    private final double long_term_avg_k = 20;
    private final double short_term_avg_k = 5;
    private double lavg, savg;
    private PriorityQueue<Double> lqueue, squeue;
    private boolean intook;

    public AutoIntakeCommand() {
        setName("auto intake");
        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
        lqueue = new PriorityQueue<>();
        squeue = new PriorityQueue<>();
        lavg = 0;
        savg = 0;
        intook = false;
    }

    @Override
    public void execute() {
//        System.out.println(StateMachine.getInstance().getProfile());
        if (OI.getInstance().getOperatorController().getRightBumper()) {
            //Intake override
            IntakeSubsystem.getInstance().setMotor(IntakeConstants.OUTTAKE_SPEED);
//            OI.getInstance().zeroAll();
            StateMachine.getInstance().setIntakeOff();
            Odometry.getInstance().setScoringCam(false);
        } else if (OI.getInstance().getOperatorController().getLeftBumper()) {

            lavg = updateAvg(lqueue, lavg, long_term_avg_k, IntakeSubsystem.getInstance().getCurrent());
            savg = updateAvg(squeue, savg, short_term_avg_k, IntakeSubsystem.getInstance().getCurrent());

            System.out.println(lavg + " " + savg);

            if(Math.abs(savg - lavg) > threshold || intook) {
                intook = true;
                IntakeSubsystem.getInstance().setMotor(0);
            } else {
                IntakeSubsystem.getInstance().setMotor(IntakeConstants.INTAKE_SPEED);
            }

            //Outtake override
//            StateMachine.getInstance().setIntakeStowing();
//            Odometry.getInstance().setScoringCam(true);
        } else if (StateMachine.getInstance().getIntakingState() == StateMachine.IntakeState.OUTTAKE) {
            //Outtake
            if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CONE) {
                IntakeSubsystem.getInstance().setMotor(IntakeConstants.OUTTAKE_SPEED_CONE);
            } else {
                IntakeSubsystem.getInstance().setMotor(IntakeConstants.OUTTAKE_SPEED);
            }
        } else if (StateMachine.getInstance().getIntakingState() == StateMachine.IntakeState.INTAKE) {
            //Intake
            IntakeSubsystem.getInstance().setMotor(IntakeConstants.INTAKE_SPEED);

//            If prox sensor detected index for another second then stow
            if(TelescopeSubsystem.getInstance().withinThreshold() && PivotSubsystem.getInstance().withinThreshold()) {
                if (IntakeSubsystem.getInstance().proxSensorTrigger() && !indexing) {
                    indexing = true;
                    Util.triggerFunctionAfterTime(() -> {
                        OI.getInstance().zeroAll();
                        StateMachine.getInstance().setIntakeStowing();
                        Odometry.getInstance().setScoringCam(true);
                        indexing = false;
                    }, 300);
                }
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

    public double updateAvg(PriorityQueue<Double> q, double avg, double k, double val) {
        if(q.size() == k) {
            avg -= 1/k * q.poll();
            avg += 1/k * val;
        } else {
            int n = q.size();
            avg = (n / (n + 1)) * avg + (1 / (n + 1)) * val;
        }
        q.add(val);
        return avg;
    }
}
