package com.github.mittyrobotics.telescope.commands;

import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.telescope.TelescopeConstants;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        if(TelescopeSubsystem.getInstance().getHalifaxMaxContact()) {
            TelescopeSubsystem.getInstance().resetMeters(TelescopeConstants.MAX_EXTENSION_METERS);
            if(TelescopeSubsystem.getInstance().getVelocityMetersPerSecond() > 0) {
                TelescopeSubsystem.getInstance().setVelZero();
            }
        } else if(TelescopeSubsystem.getInstance().getHalifaxMinContact()) {
            TelescopeSubsystem.getInstance().resetMeters(0);
            if(TelescopeSubsystem.getInstance().getVelocityMetersPerSecond() < 0) {
                TelescopeSubsystem.getInstance().setVelZero();
            }
        }

        TelescopeSubsystem.getInstance().setPositionMeters(ArmKinematics.getTelescopeDesired());
        System.out.println("EXTENSION INCHES: " + TelescopeSubsystem.getInstance().getDistanceInches());
        SmartDashboard.putNumber("EXTENSION INCHES", TelescopeSubsystem.getInstance().getDistanceInches());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
