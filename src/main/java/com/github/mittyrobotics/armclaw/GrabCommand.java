package com.github.mittyrobotics.armclaw;

import com.github.mittyrobotics.util.OI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class GrabCommand extends InstantCommand {
    private double angleDegrees;
    private boolean isCone;

    public GrabCommand(boolean isCone) {
//        addRequirements(ClawGrabberSubsystem.getInstance());
//        this.isCone=isCone;
//        if(isCone){
//            angleDegrees=ClawConstants.CONE_DEGREES;
//        }
//        else{
//            angleDegrees=ClawConstants.CUBE_DEGREES;
//        }
        super(() -> ClawGrabberSubsystem.getInstance().setGrabberAngle(isCone ? ClawConstants.CONE_DEGREES : ClawConstants.CUBE_DEGREES));
    }
}
