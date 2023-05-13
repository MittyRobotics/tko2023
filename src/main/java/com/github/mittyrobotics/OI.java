package com.github.mittyrobotics;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {
    private static OI instance;
    private XboxController xboxController;
    private XboxController xboxController2;
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

    public XboxController getXboxController2() {
        if (xboxController2 == null) {
            xboxController2 = new XboxController(4);
        }
        return xboxController2;
    }
    public double rjoystick()
    {
        return xboxController.getRightY();
    }

    public double ljoystick()
    {
        return xboxController.getLeftY();
    }




    public double controls_intake() {
        return xboxController2.getLeftY();
    }
    public double controls_outake(){
        return xboxController2.getRightY();
    }


}

