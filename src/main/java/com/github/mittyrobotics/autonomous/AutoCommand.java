package com.github.mittyrobotics.autonomous;

import com.github.mittyrobotics.OI;
import com.github.mittyrobotics.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoCommand extends CommandBase {

    public AutoCommand() {
        addRequirements(DriveTrainSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        System.out.println("auto command has been initialized");

        //ensures drivetrain motors are off before beginning auto
        DriveTrainSubsystem.getInstance().setMotors( 0, 0);


        //ensures roller is off before beginning auto
        DriveTrainSubsystem.getInstance().setRoller( 0);
    }

    @Override
    public void execute() {

        System.out.println("auto command is executing");
        new WaitCommand(5); //waits 5 seconds

        //while the robot has driven less than 4 feet(48 inches), drive forward
        while(DriveTrainSubsystem.getInstance().getPosition() < 48){
            DriveTrainSubsystem.getInstance().setMotors(0.5, 0.5);
        }


        //shoot ball
        DriveTrainSubsystem.getInstance().setRoller(-0.5);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
