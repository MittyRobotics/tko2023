package com.github.mittyrobotics.telescope.commands;

import com.github.mittyrobotics.intake.IntakeSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeConstants;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtensionToKinematics extends CommandBase {

    TrapezoidalMotionProfile tpTelescope;
    double velTelescope, lastTime;
    public ExtensionToKinematics() {
        super();
        addRequirements(TelescopeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        tpTelescope = new TrapezoidalMotionProfile(40 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 120 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0, 0, 0 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 20 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV, 0.2);
        velTelescope = 0;
        lastTime = Timer.getFPGATimestamp();

    }

    @Override
    public void execute() {
//        if(TelescopeSubsystem.getInstance().getHalifaxMaxContact()) {
//            TelescopeSubsystem.getInstance().resetMeters(TelescopeConstants.MAX_EXTENSION_METERS);
//        } else if(TelescopeSubsystem.getInstance().getHalifaxMinContact()) {
//            TelescopeSubsystem.getInstance().resetMeters(0);
//        }

        double desired = (ArmKinematics.getTelescopeDesired() +
                (StateMachine.getInstance().getIntakingState() == StateMachine.IntakeState.INTAKE ? 1 / 39.37 : 0)) / TelescopeConstants.METERS_PER_MOTOR_REV;
        tpTelescope.changeSetpoint(desired, TelescopeSubsystem.getInstance().rawPos(), TelescopeSubsystem.getInstance().rawVel() / 60);

        boolean telescopeMovingDown = tpTelescope.getSetpoint() < TelescopeSubsystem.getInstance().rawPos();

        double pidmax = 0.00025;
        double telescopeP = Math.max(0.0001, pidmax - (pidmax - 0.0001) * Math.sin(PivotSubsystem.getInstance().getPositionRadians()));
        TelescopeSubsystem.getInstance().setPID(telescopeP <= pidmax ? telescopeP : 0, 0, 0);

        double telescopeFF = (0.25 / (300 + (900 - 300) * Math.pow(Math.sin(PivotSubsystem.getInstance().getPositionRadians()), 6))) *
                (telescopeMovingDown ? 1 - 0.85 * Math.cos(PivotSubsystem.getInstance().getPositionRadians()) : 1.4);
        TelescopeSubsystem.getInstance().setFF(telescopeFF);

        velTelescope = 0;
        velTelescope = 60 * tpTelescope.update(Timer.getFPGATimestamp() - lastTime, TelescopeSubsystem.getInstance().rawPos());

        TelescopeSubsystem.getInstance().setRaw(velTelescope);

        lastTime = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("Telescope meters", TelescopeSubsystem.getInstance().getDistanceMeters());
        SmartDashboard.putNumber("Telescope vel", TelescopeSubsystem.getInstance().rawVel());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
