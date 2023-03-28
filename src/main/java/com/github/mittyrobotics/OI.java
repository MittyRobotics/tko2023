package com.github.mittyrobotics;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class OI {
    private static OI instance;
    private XboxController xboxController;

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }


    public XboxController getXboxController() {
        if (xboxController == null) {
            xboxController = new XboxController(3);
        }
        return xboxController;
    }
    public double rjoystick()
    {
        return xboxController.getRightX();
    }

    public double ljoystick()
    {
        return xboxController.getLeftY();
    }


    /*public void setIntaking(boolean a)
    {
        if(a == false){IntakeSystem.getInstance().stop();}
    }*/

    /*public void setOutaking(boolean a)
    {
        if(a == false){OutakeSystem.getInstance().stop();}
    }
*/

    public void controls() {
        /*Button intake = new Button(() -> getXboxController().getAButton());
        intake.whenPressed(new IntakeCommand());
        Button outake = new Button(() -> getXboxController().getBButton());
        outake.whenPressed(new OutakeCommand());*/
    }


}

