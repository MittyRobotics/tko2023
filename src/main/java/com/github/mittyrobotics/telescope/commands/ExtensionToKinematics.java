package com.github.mittyrobotics.telescope.commands;

import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtensionToKinematics extends CommandBase {

    public ExtensionToKinematics() {
        super();
        addRequirements(TelescopeSubsystem.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        TelescopeSubsystem.getInstance().setPositionMeters(ArmKinematics.getTelescopeDesired());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
