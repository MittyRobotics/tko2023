package com.github.mittyrobotics;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainCommand2 extends CommandBase {

    private boolean flag = true;

    public DriveTrainCommand2(){addRequirements(DriveTrainSystem.getInstance());}
    @Override
    public void initialize(){
        DriveTrainSystem.getInstance().encoder_value();

    }
    @Override
    public void execute(){
        DriveTrainSystem.getInstance().aa();
        //DriveTrainSystem.getInstance().aa();


        //check drivetrain system
    }
    @Override
    public void end(boolean interrupted){
        System.out.println("________________________________done");
        DriveTrainSystem.getInstance().stop();
        flag= false;
    }
    @Override
    public boolean isFinished(){
        if(DriveTrainSystem.getInstance().aa() == true && flag)
        {
            //DriveTrainSystem.getInstance().stop();
            return true;
        }
        return false;
    }
}
