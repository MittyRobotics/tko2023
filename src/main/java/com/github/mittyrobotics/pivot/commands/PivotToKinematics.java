package com.github.mittyrobotics.pivot.commands;

import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotConstants;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        if(PivotSubsystem.getInstance().getHalifaxTopContact()) {
            PivotSubsystem.getInstance().resetAngleDegrees(PivotConstants.HALIFAX_TOP_DEGREES);
        } else if(PivotSubsystem.getInstance().getHalifaxBottomContact()) {
            PivotSubsystem.getInstance().resetAngleDegrees(PivotConstants.HALIFAX_BOTTOM_DEGREES);
        }

        if(PivotSubsystem.getInstance().getPositionRadians() > PivotConstants.SOFT_LIMIT_TOP_RADIANS) {
            PivotSubsystem.getInstance().setVelZero();
        } else if(PivotSubsystem.getInstance().getPositionRadians() < PivotConstants.SOFT_LIMIT_BOTTOM_RADIANS) {
            PivotSubsystem.getInstance().setVelZero();
        }

        System.out.println("PIVOT DEGREES: " + PivotSubsystem.getInstance().getPositionDegrees());
        SmartDashboard.putNumber("PIVOT DEGREES", PivotSubsystem.getInstance().getPositionDegrees());

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
