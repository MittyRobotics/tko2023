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
        IntakeSystem.getInstance().setIntakeDown();
    }

    public void execute() {

        IntakeSystem.getInstance().run();
    }
    @Override
    public void end(boolean interrupted) {
        IntakeSystem.getInstance().setIntakeUp();
        OI.getInstance().setIntaking(false);
    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
