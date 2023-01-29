package com.github.mittyrobotics.pivot.commands;

import com.github.mittyrobotics.pivot.ArmKinematics;
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
//        if(PivotSubsystem.getInstance().getHalifaxTopContact()) {
//            if(PivotSubsystem.getInstance().getVelocityDegreesPerSecond() > 0) {
//                PivotSubsystem.getInstance().resetAngleDegrees(9.033033020642339);
//            } else if(PivotSubsystem.getInstance().getVelocityDegreesPerSecond() < 0) {
//                PivotSubsystem.getInstance().resetAngleDegrees(5.024293928730245);
//            }
//        } else if(PivotSubsystem.getInstance().getHalifaxBottomContact()) {
//            if(PivotSubsystem.getInstance().getVelocityDegreesPerSecond() < 0) {
//                PivotSubsystem.getInstance().resetAngleDegrees(-9.033033020642339);
//            } else if(PivotSubsystem.getInstance().getVelocityDegreesPerSecond() > 0) {
//                PivotSubsystem.getInstance().resetAngleDegrees(-5.024293928730245);
//            }
//        }

//        System.out.println("PIVOT DES: " + ArmKinematics.getPivotDesiredPolar());
//        System.out.println("PIVOT DEGREES: " + PivotSubsystem.getInstance().getPositionDegrees());
//        SmartDashboard.putNumber("PIVOT DEGREES", PivotSubsystem.getInstance().getPositionDegrees());

        double desired = ArmKinematics.getPivotDesiredPolar().getRadians();
        double currentExtension = TelescopeSubsystem.getInstance().getDistanceMeters();
//        PivotSubsystem.getInstance().configPID(
//                PivotConstants.PIVOT_BASE_P * currentExtension * currentExtension,
//                PivotConstants.PIVOT_BASE_I * currentExtension * currentExtension,
//                PivotConstants.PIVOT_BASE_D * currentExtension * currentExtension);
        PivotSubsystem.getInstance().configPID(0.1, 0, 0);
        if(Math.PI/2 > Math.abs(desired)) {
            PivotSubsystem.getInstance().setPositionRadians(desired);
        } else {
            PivotSubsystem.getInstance().setPositionRadians(desired > 0 ? Math.PI/2 : - Math.PI/2);
        }

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
