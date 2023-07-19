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
        System.out.println("intake command has been initialized");

        DriveTrainSubsystem.getInstance().setRoller(0);
    }

    @Override
    public void execute() {

        System.out.println("intake command is executing");
        if (OI.getInstance().getOperatorController().getCircleButton()) {
            DriveTrainSubsystem.getInstance().setRoller(0.5);
        } else if(OI.getInstance().getOperatorController().getCrossButton()){DriveTrainSubsystem.getInstance().setRoller(-0.5);} else {
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
