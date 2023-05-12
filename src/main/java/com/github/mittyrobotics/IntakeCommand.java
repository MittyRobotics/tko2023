package com.github.mittyrobotics;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {
    public IntakeCommand() {
        addRequirements(IntakeSystem.getInstance());
    }

    @Override
    public void initialize()
    {
        //set up OI - naomi
        OI.getInstance();

    }

    public void execute() {

        IntakeSystem.getInstance().periodic();
    }
    @Override
    public void end(boolean interrupted) {
    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
