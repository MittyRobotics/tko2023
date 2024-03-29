package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FastOvershootBalance extends CommandBase {
    private double maxVelStart, maxVelBalance;
    private boolean onScale = false;
    private boolean docking = false;

    private final double STOP_ANGLE = 8;
    private final double ON_ANGLE = 14;
    private final double BACK_ANGLE = 12
            ;
    private final boolean pos;

    public FastOvershootBalance(double maxVelStart, double maxVelBalance, boolean pos) {
        this.maxVelBalance = maxVelBalance;
        this.maxVelStart = maxVelStart;
        this.pos = pos;

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        onScale = false;
        docking = false;
    }

    @Override
    public void execute() {
        double pitch = Math.abs(Gyro.getInstance().getPitch());
        double speed;

        LoggerInterface.getInstance().put("Pitch", Gyro.getInstance().getPitch());


        if (!onScale) {
            if (pitch > ON_ANGLE) onScale = true;
            speed = maxVelStart;
        } else {
            if (pitch < BACK_ANGLE) docking = true;
            if(docking) {
                speed = -maxVelBalance;
            } else {
                speed = maxVelStart;
            }
        }
        SwerveSubsystem.getInstance().setSwerveInvKinematics(new Vector(
                pos ? speed : -speed, 0), 0);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setZero();
        SwerveSubsystem.getInstance().fortyFiveAngle();
    }

    @Override
    public boolean isFinished() {
//        System.out.println(docking);
        return docking && Math.abs(Gyro.getInstance().getPitch()) < STOP_ANGLE;
    }
}
