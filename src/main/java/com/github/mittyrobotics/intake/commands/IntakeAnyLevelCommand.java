package com.github.mittyrobotics.intake.commands;

import com.github.mittyrobotics.intake.ClawGrabberSubsystem;
import com.github.mittyrobotics.intake.ClawRollerSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.PriorityQueue;

public class IntakeAnyLevelCommand extends CommandBase {

    private final double threshold = 15;

    private final double long_term_avg_k = 20;
    private final double short_term_avg_k = 5;
    private double lavg, savg;
    private PriorityQueue<Double> lqueue, squeue;
    private boolean intook;

    @Override
    public void initialize() {
        lqueue = new PriorityQueue<>();
        squeue = new PriorityQueue<>();
        lavg = 0;
        savg = 0;
        intook = false;
    }

    @Override
    public void execute() {
        lavg = updateAvg(lqueue, lavg, long_term_avg_k, ClawRollerSubsystem.getInstance().getCurrent());
        savg = updateAvg(squeue, savg, short_term_avg_k, ClawRollerSubsystem.getInstance().getCurrent());

        if(lavg - savg > threshold || intook) {
            intook = true;
            ClawRollerSubsystem.getInstance().setRoller(0);
        } else {
            ClawRollerSubsystem.getInstance().setRoller(-0.4);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ClawRollerSubsystem.getInstance().setRoller(0);
    }

    @Override
    public boolean isFinished() {
        return OI.getInstance().getOperatorController().getLeftTriggerAxis() <= 0.5;
    }

    public double updateAvg(PriorityQueue<Double> q, double avg, double k, double val) {
        if(q.size() == k) {
            avg -= 1/k * q.poll();
            avg += 1/k * val;
        } else {
            int n = q.size();
            avg = ((double) n / (n + 1)) * avg + (1. / (n + 1)) * val;
        }
        q.add(val);
        return avg;
    }
}
