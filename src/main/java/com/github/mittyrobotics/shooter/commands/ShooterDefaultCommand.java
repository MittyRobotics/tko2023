package com.github.mittyrobotics.shooter.commands;

import com.github.mittyrobotics.shooter.ShooterConstants;
import com.github.mittyrobotics.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterDefaultCommand extends CommandBase {
    public ShooterDefaultCommand() {
        addRequirements(ShooterSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (ShooterSubsystem.getInstance().isShooting()) {
            ShooterSubsystem.getInstance().setSpeed(ShooterConstants.SHOOTER_RPM);
        } else {
            ShooterSubsystem.getInstance().setSpeed(0);
            ShooterSubsystem.getInstance().setMotor(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
