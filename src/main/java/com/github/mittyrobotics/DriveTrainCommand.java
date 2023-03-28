package com.github.mittyrobotics;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainCommand extends CommandBase {
    public DriveTrainCommand(){addRequirements(DriveTrainSystem.getInstance());}

    @Override
    public void initialize(){
        DriveTrainSystem.getInstance().periodic();

    }
    @Override
    public void execute(){

        DriveTrainSystem.getInstance().run();

        //check drivetrain system
    }
    @Override
    public void end(boolean interrupted){
        DriveTrainSystem.getInstance().periodic();
    }
    @Override
    public boolean isFinished(){

        return false;
    }
}