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

        DriveTrainSystem.getInstance().executePID();
        //DriveTrainSystem.getInstance().aa();


        //check drivetrain system
    }
    @Override
    public void end(boolean interrupted){
        //DriveTrainSystem.getInstance().periodic();
        System.out.println("__________________done");
    }
    @Override
    public boolean isFinished(){
        if(DriveTrainSystem.getInstance().executePID() == true)
        {
            //System.out.println("PID" + DriveTrainSystem.getInstance().executePID());
            return true;
        }
        return false;
    }
}