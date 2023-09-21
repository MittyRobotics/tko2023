package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.util.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static java.lang.Math.PI;

public class TimedOvershootBalance extends CommandBase {
    private double maxVelStart, maxVelBalance;
    private boolean forward = true;
    private boolean waiting = true;

    private final double STOP_ANGLE = 10;
    private final double ON_ANGLE = 5;

    private final long time;
    private final boolean pos;
    private boolean index, index2;

    public TimedOvershootBalance(double maxVelStart, long time, double maxVelBalance, boolean pos) {
        this.maxVelBalance = maxVelBalance;
        this.maxVelStart = maxVelStart;
        this.pos = pos;
        this.time = time;

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        forward = true;
        waiting = true;
        index = false;
        index2 = false;
    }

    @Override
    public void execute() {
        double pitch = Math.abs(Gyro.getInstance().getPitch());
        double speed;

        if(forward) {
            speed = maxVelStart;

            if(pitch > ON_ANGLE && !index) {
                Util.triggerFunctionAfterTime(() -> {
                    forward = false;
                    index = true;
                }, time);
            }
        } else {
            if(!index2)
                Util.triggerFunctionAfterTime(() -> {
                    waiting = false;
                    index2 = true;
                }, 1000);
            if (!waiting) speed = -maxVelBalance;
            else speed = 0;
        }

        SwerveSubsystem.getInstance().calculateInputs(new Vector(
                pos ? speed : -speed, 0), 0);

        SwerveSubsystem.getInstance().applyCalculatedInputs();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setDriveMotors(new double[] {0, 0, 0, 0});
        SwerveSubsystem.getInstance().setAngleMotors(new double[] {PI / 4, PI / 4, PI / 4, PI / 4});
    }

    @Override
    public boolean isFinished() {
        return !waiting && Math.abs(Gyro.getInstance().getPitch()) < STOP_ANGLE;
    }
}
