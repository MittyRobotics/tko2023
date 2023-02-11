package com.github.mittyrobotics.pivot.commands;

import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotConstants;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotToKinematics extends CommandBase {
    TrapezoidalMotionProfile tpPivot;
    double lastTime;
    public PivotToKinematics() {
        super();
        addRequirements(PivotSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        tpPivot = new TrapezoidalMotionProfile(360 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 360 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 360 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0, 0, PivotSubsystem.getInstance().rawPos(), 20 / 360. / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO, 0.3);
        lastTime = Timer.getFPGATimestamp();
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

        double desired = ArmKinematics.getPivotDesiredPolar().getRadians();

        PivotSubsystem.getInstance().configPID(PivotConstants.PIVOT_BASE_P, PivotConstants.PIVOT_BASE_I, PivotConstants.PIVOT_BASE_D);
        PivotSubsystem.getInstance().setFF(PivotConstants.PIVOT_FF);

        tpPivot.changeSetpoint(desired, PivotSubsystem.getInstance().rawPos(), PivotSubsystem.getInstance().rawVel());
        double velPivot = 60 * tpPivot.update(Timer.getFPGATimestamp() - lastTime, PivotSubsystem.getInstance().rawPos());

        PivotSubsystem.getInstance().setRaw(velPivot);

        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
