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
        if(PivotSubsystem.getInstance().getHalifaxContact()) {
            PivotSubsystem.getInstance().resetAngleDegrees(PivotConstants.HALIFAX_POSITION_DEGREES);
        }

        double desired = ArmKinematics.getPivotDesired().getRadians();
        double currentExtension = TelescopeSubsystem.getInstance().getDistanceMeters();
        PivotSubsystem.getInstance().configPID(
                PivotConstants.PIVOT_BASE_P * currentExtension * currentExtension,
                PivotConstants.PIVOT_BASE_I * currentExtension * currentExtension,
                PivotConstants.PIVOT_BASE_D * currentExtension * currentExtension);
        PivotSubsystem.getInstance().setPositionRadians(Math.abs(desired) < Math.PI/4 ? desired : 0);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
