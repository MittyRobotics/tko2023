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
      //  System.out.println("intake command has been initialized");

        DriveTrainSubsystem.getInstance().setRoller(0);
    }

    @Override
    public void execute() {

     //   System.out.println("intake command is executing");

            DriveTrainSubsystem.getInstance().setRoller(0.5);

    }

    @Override
    public void end(boolean interrupted) {
        DriveTrainSubsystem.getInstance().setRoller(0);


    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
