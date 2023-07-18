package com.github.mittyrobotics.commands;

import com.github.mittyrobotics.OI;
import com.github.mittyrobotics.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveCommand extends CommandBase {
    /********************************************
     *** HIT BOTH TRIGGERS TO EXIT TANK DRIVE ***
     ********************************************/

    private double leftPower, rightPower;
    private boolean isRunning;

    public TankDriveCommand() {
        addRequirements(DriveTrainSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        DriveTrainSubsystem.getInstance().setMotors(0, 0);
    }

    @Override
    public void execute() {
        double percent = 0.5;
        double deadzone = 0.1;
        double left = OI.getInstance().getXboxController().getLeftY()* percent;
        double right = OI.getInstance().getXboxController().getRightY() * percent;
        if(Math.abs(left) < deadzone) {
            left = 0;
        }
        if(Math.abs(right) < deadzone) {
            right = 0;
        }
        DriveTrainSubsystem.getInstance().setMotors(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        DriveTrainSubsystem.getInstance().setMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
