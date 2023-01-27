package com.github.mittyrobotics.pivot.commands;

import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotConstants;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotToKinematics extends CommandBase {

    public PivotToKinematics() {
        super();
        addRequirements(PivotSubsystem.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double desired = ArmKinematics.getPivotDesired().getRadians();
        double currentExtension = TelescopeSubsystem.getInstance().getDistanceMeters();
        PivotSubsystem.getInstance().configPID(PivotConstants.PIVOT_BASE_P * currentExtension,
                PivotConstants.PIVOT_BASE_I * currentExtension, PivotConstants.PIVOT_BASE_D * currentExtension);
        PivotSubsystem.getInstance().setPositionRadians(desired);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
