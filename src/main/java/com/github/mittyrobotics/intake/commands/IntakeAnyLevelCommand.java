package com.github.mittyrobotics.intake.commands;

import com.github.mittyrobotics.intake.ClawGrabberSubsystem;
import com.github.mittyrobotics.intake.ClawRollerSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeAnyLevelCommand extends CommandBase {

    private boolean isLoading;

    @Override
    public void initialize() {
        isLoading = ClawGrabberSubsystem.getInstance().getProximitySensor();
    }

    @Override
    public void execute() {
        isLoading = ClawGrabberSubsystem.getInstance().getProximitySensor();
        ClawRollerSubsystem.getInstance().handleGamePiece(isLoading);
    }

    @Override
    public void end(boolean interrupted) {
        ClawRollerSubsystem.getInstance().setRoller(0);
    }

    @Override
    public boolean isFinished() {
        //update with all other levels TODO
        return !(OI.getInstance().getOperatorController().getAButton() ||
                OI.getInstance().getOperatorController().getBButton() ||
                OI.getInstance().getOperatorController().getXButton() ||
                OI.getInstance().getOperatorController().getYButton());
    }
}
