package com.github.mittyrobotics.conveyor.commands;

import com.github.mittyrobotics.conveyor.ConveyorSubsystem;
import com.github.mittyrobotics.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorDefaultCommand extends CommandBase {
    public ConveyorDefaultCommand() {
        addRequirements(ConveyorSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (ConveyorSubsystem.getInstance().isIntaking()) {
            if (ConveyorSubsystem.getInstance().getLimitSwitchTripped() &&
                    !ShooterSubsystem.getInstance().isShooting()) {
                ConveyorSubsystem.getInstance().stopIntaking();
            } else {
                ConveyorSubsystem.getInstance().startIntaking();
                ConveyorSubsystem.getInstance().setMotor(0.5);
            }
        }

        if (!ConveyorSubsystem.getInstance().isIntaking()) {
            ConveyorSubsystem.getInstance().setMotor(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ConveyorSubsystem.getInstance().setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
