package com.github.mittyrobotics;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainCommand2 extends CommandBase {

    public DriveTrainCommand2(){addRequirements(DriveTrainSystem.getInstance());}
    @Override
    public void initialize(){
        DriveTrainSystem.getInstance().aa();

    }
    @Override
    public void execute(){

        DriveTrainSystem.getInstance().aa();
        //DriveTrainSystem.getInstance().aa();


        //check drivetrain system
    }
    @Override
    public void end(boolean interrupted){
        DriveTrainSystem.getInstance().aa();
    }
    @Override
    public boolean isFinished(){
        if(DriveTrainSystem.getInstance().getencoders(30) == true)
        {
            return true;
        }
        return false;
    }
}
