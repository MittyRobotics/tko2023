package com.github.mittyrobotics.pivot.commands;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotConstants;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeConstants;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotToKinematics extends CommandBase {

    TrapezoidalMotionProfile tpPivot;
    double velPivot, lastTime;

    public PivotToKinematics() {
        super();
        addRequirements(PivotSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        velPivot = 0;
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
        tpPivot = PivotConstants.PIVOT_MPS.get(StateMachine.getInstance().getProfile());

        double desired = ArmKinematics.getPivotDesired().getRadians() / (2 * Math.PI) / PivotConstants.PIVOT_TO_NEO_GEAR_RATIO;
        tpPivot.changeSetpoint(desired, PivotSubsystem.getInstance().rawPos(), PivotSubsystem.getInstance().rawVel() / 60);

        double currentExtension = TelescopeSubsystem.getInstance().getDistanceInches();

        boolean pivotMovingDown = tpPivot.getSetpoint() > PivotSubsystem.getInstance().rawPos();

        double pivotFF = 0.3/(1765.) * (pivotMovingDown ?
                0.6 - 0 / 12. * currentExtension :
                0.8 + 0.2 / 12. * currentExtension);
        PivotSubsystem.getInstance().setFF(pivotFF);

        PivotSubsystem.getInstance().configPID(0.00002, 0, 0);

        tpPivot.setMinOutput(30 / 39.37 / TelescopeConstants.METERS_PER_MOTOR_REV * (pivotMovingDown ?
                1 - 0 / 12. * currentExtension :
                1 + 0.1 / 12. * currentExtension));

        velPivot = 0;
//        if(OI.getInstance().getOperatorController().getRightTriggerAxis() > 0.2)
            velPivot = 60 * tpPivot.update(Timer.getFPGATimestamp() - lastTime, PivotSubsystem.getInstance().rawPos());

//        PivotSubsystem.getInstance().setRaw(OI.getInstance().getOperatorController().getRightTriggerAxis() > 0.2 ? velPivot : 0);
        PivotSubsystem.getInstance().setRaw(velPivot);

        lastTime = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("Pivot Radians", PivotSubsystem.getInstance().getPositionRadians());
        SmartDashboard.putNumber("Pivot raw vel", PivotSubsystem.getInstance().rawVel());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
