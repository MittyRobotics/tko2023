package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedBangBangBalance extends CommandBase {
    private double maxVelStart, maxVelBalance;
    private boolean forward = true;

    private final double STOP_ANGLE = 8;
    private final double ON_ANGLE = 10;

    private final long time;
    private final boolean pos;
    private boolean index;

    public TimedBangBangBalance(double maxVelStart, long time, double maxVelBalance, boolean pos) {
        this.maxVelBalance = maxVelBalance;
        this.maxVelStart = maxVelStart;
        this.pos = pos;
        this.time = time;

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        forward = true;
        index = false;
    }

    @Override
    public void execute() {
        double pitch = Gyro.getInstance().getPitch();
        double speed;

        if(forward) {
            speed = pos ? maxVelStart : -maxVelStart;

            if(Math.abs(pitch) > ON_ANGLE && !index) {
                index = true;
                Util.triggerFunctionAfterTime(() -> {
                    forward = false;
                }, time);
            }
        } else {
            if (Math.abs(pitch) < STOP_ANGLE) {
                SwerveSubsystem.getInstance().fortyFiveAngle();
                speed = 0;
            }
            else speed = pitch < 0 ? maxVelBalance : -maxVelBalance;
        }

        double heading = Gyro.getInstance().getHeadingRadians();
        double desAngle;
        if (Odometry.getInstance().FIELD_LEFT_SIDE) {
            desAngle = (speed < 0) ? 0 : Math.PI;
        } else {
            desAngle = (speed < 0) ? Math.PI : 0;
        }

        double ang = desAngle - heading;
        Vector linear = new Vector(new Angle(ang), Math.abs(speed));

        SwerveSubsystem.getInstance().setSwerveInvKinematics(linear, 0);

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
        return false;
    }
}
