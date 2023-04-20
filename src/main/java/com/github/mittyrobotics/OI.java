package com.github.mittyrobotics;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        return xboxController.getRightY();
    }

    public double ljoystick()
    {
        return xboxController.getLeftY();
    }




    public void controls() {
        Trigger intake = new JoystickButton(xboxController, XboxController.Button.kY.value)
                .whileTrue(new IntakeCommand());
                //intake.onFalse(OutakeSystem.getInstance().stop());
        Trigger outake = new JoystickButton(xboxController, XboxController.Button.kX.value)
                .whileTrue(new OutakeCommand());
    }


}

