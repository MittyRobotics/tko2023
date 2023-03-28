package com.github.mittyrobotics;


import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.*;



public class Robot extends TimedRobot {

    @Override
    public void robotInit() {
        IntakeSystem.getInstance();
        IntakeSystem.getInstance().initHardware();
        //OutakeSystem.getInstance();
        //OutakeSystem.getInstance().initHardware();
        OI.getInstance();
        DriveTrainSystem.getInstance();
        DriveTrainSystem.getInstance().initHardware();

        //PID.initHardware();
        //TODO code outake system
    }

    @Override
    public void robotPeriodic() {
        DriveTrainSystem.getInstance().run();

    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        OI.getInstance().controls();

    }

    @Override
    public void teleopPeriodic() {
        //call drive train system


        //TODO create OI function

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {
    }
}
