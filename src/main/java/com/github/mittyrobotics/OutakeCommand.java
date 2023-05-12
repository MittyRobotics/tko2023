package com.github.mittyrobotics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class OutakeCommand extends CommandBase {
    public OutakeCommand() {
        addRequirements(OutakeSystem.getInstance());
    }

    @Override
    public void initialize()
    {
        //set up OI - naomi
        OI.getInstance();
        OutakeSystem.getInstance();
    }

    public void execute() {

        OutakeSystem.getInstance().periodic();
    }
    @Override
    public void end(boolean interrupted) {
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
