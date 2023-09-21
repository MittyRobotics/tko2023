package com.github.mittyrobotics.arm.televator.commands;

import com.github.mittyrobotics.arm.ArmKinematics;
import com.github.mittyrobotics.arm.ArmMotionProfiles;
import com.github.mittyrobotics.arm.StateMachine;
import com.github.mittyrobotics.arm.pivot.PivotSubsystem;
import com.github.mittyrobotics.arm.televator.TelevatorSubsystem;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TelevatorDefaultCommand extends CommandBase {
    public TelevatorDefaultCommand() {
        setName("Move To Desired Extension");
        addRequirements(TelevatorSubsystem.getInstance());
    }

    private double desiredExtension, desiredVel, ff, dt, time;
    private TrapezoidalMotionProfile motionProfile;

    @Override
    public void initialize() {
        desiredExtension = 0;
    }

    @Override
    public void execute() {
        dt = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();

        desiredExtension = ArmKinematics.getDesiredExtension();

        motionProfile = ArmMotionProfiles.TELEVATOR_MPS.get(StateMachine.getTransitionState());
        motionProfile.changeSetpoint(
                desiredExtension,
                TelevatorSubsystem.getInstance().getCurrentExtension(),
                TelevatorSubsystem.getInstance().getCurrentVelocity()
        );

        double currentExtension = TelevatorSubsystem.getInstance().getCurrentExtension();
        desiredVel = motionProfile.update(dt, currentExtension);

        boolean telescopeMovingDown = motionProfile.getSetpoint() < currentExtension;

        // TODO: 6/27/2023 ADD FF STUFF
        ff = (0.2 / (300 + (500 - 300) * Math.pow(Math.sin(PivotSubsystem.getInstance().getCurrentAngle().getRadians()), 6))) *
                (telescopeMovingDown ? 1 - 0.8 * Math.cos(PivotSubsystem.getInstance().getCurrentAngle().getRadians()) : 1.75);


        TelevatorSubsystem.getInstance().setVelArbFF(desiredVel, ff);
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
