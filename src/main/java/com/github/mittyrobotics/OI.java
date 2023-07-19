package com.github.mittyrobotics;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
    private static OI instance;

    private XboxController controller;

    private PS4Controller operatorController;

    public static OI getInstance() {
        if(instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public void initOI() {
        controller = new XboxController(0);
        operatorController = new PS4Controller(1);
    }

    public XboxController getXboxController() {
        return controller;
    }
    public PS4Controller getOperatorController() {return operatorController;}


}