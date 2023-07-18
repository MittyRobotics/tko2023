package com.github.mittyrobotics.commands;

import com.github.mittyrobotics.OI;
import com.github.mittyrobotics.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {
    /********************************************
     *** HIT BOTH TRIGGERS TO EXIT TANK DRIVE ***
     ********************************************/

    private double leftPower, rightPower;
    private boolean isRunning;

    public IntakeCommand() {
        addRequirements(DriveTrainSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        DriveTrainSubsystem.getInstance().setRoller(0);
    }

    @Override
    public void execute() {
        if (OI.getInstance().getXboxController().getAButton()) {
            DriveTrainSubsystem.getInstance().setRoller(0.5);
        } else if(OI.getInstance().getXboxController().getBButton()){DriveTrainSubsystem.getInstance().setRoller(-0.5);} else {
            DriveTrainSubsystem.getInstance().setRoller(0);

        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
