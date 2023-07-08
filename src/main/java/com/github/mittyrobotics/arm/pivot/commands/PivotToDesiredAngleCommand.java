package com.github.mittyrobotics.arm.pivot.commands;

import com.github.mittyrobotics.arm.ArmKinematics;
import com.github.mittyrobotics.arm.MotionProfiles;
import com.github.mittyrobotics.arm.StateMachine;
import com.github.mittyrobotics.arm.pivot.PivotSubsystem;
import com.github.mittyrobotics.arm.televator.TelevatorSubsystem;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotToDesiredAngleCommand extends CommandBase {
    public PivotToDesiredAngleCommand() {
        setName("Pivot To Desired Angle");
        addRequirements(PivotSubsystem.getInstance());
    }

    private double desiredAngle, desiredRPM, ff, dt, time;
    private TrapezoidalMotionProfile motionProfile;

    @Override
    public void initialize() {
        desiredAngle = 0;
        time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        dt = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();

        desiredAngle = ArmKinematics.getDesiredAngle().getRadians();

        // TODO: 6/27/2023 ADD MP STUFF
        motionProfile = MotionProfiles.PIVOT_MPS.get(StateMachine.getTransitionState());
        motionProfile.changeSetpoint(
                desiredAngle,
                PivotSubsystem.getInstance().getCurrentAngle().getRadians(),
                PivotSubsystem.getInstance().getCurrentVelocity()
        );

        desiredRPM = motionProfile.update(dt, PivotSubsystem.getInstance().getCurrentAngle().getRadians());

        boolean pivotMovingDown = PivotSubsystem.getInstance().getCurrentVelocity() > 0;
        double currentExtension = TelevatorSubsystem.getInstance().getCurrentExtension();

        // TODO: 6/27/2023 ADD FF STUFF
        ff = 0.3/(1765) * (pivotMovingDown ?
                1 - 0 / 12. * currentExtension :
                1.2 + 0.2 / 12. * currentExtension);

        PivotSubsystem.getInstance().setRaw(desiredRPM, ff);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
