package com.github.mittyrobotics.arm.televator.commands;

import com.github.mittyrobotics.arm.ArmKinematics;
import com.github.mittyrobotics.arm.MotionProfiles;
import com.github.mittyrobotics.arm.StateMachine;
import com.github.mittyrobotics.arm.pivot.PivotSubsystem;
import com.github.mittyrobotics.arm.televator.TelevatorSubsystem;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToDesiredExtensionCommand extends CommandBase {
    public MoveToDesiredExtensionCommand() {
        setName("Move To Desired Extension");
        addRequirements(TelevatorSubsystem.getInstance());
    }

    private double desiredExtension, desiredVel, ff;
    private TrapezoidalMotionProfile motionProfile;

    @Override
    public void initialize() {
        desiredExtension = 0;
    }

    @Override
    public void execute() {
        desiredExtension = ArmKinematics.getDesiredExtension();

        // TODO: 6/27/2023 ADD MP STUFF
        motionProfile = MotionProfiles.TELEVATOR_MPS.get(StateMachine.getTransitionState());

        // TODO: 6/27/2023 ADD FF STUFF

        TelevatorSubsystem.getInstance().setRaw(desiredVel, ff);
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
