/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.util;

import com.github.mittyrobotics.StateMachine;
import com.github.mittyrobotics.drivetrain.commands.SnapToAngle;
import com.github.mittyrobotics.intake.commands.IntakeAnyLevelCommand;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * OI Class to manage all controllers and input
 */
public class OI {
    private static OI instance;

    private PS4Controller driverController;

    private XboxController throttleWheel;
    private XboxController steeringWheel;
    private XboxController operatorController;
    private XboxController driveTestingController;

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public XboxController getThrottleWheel() {
        if (throttleWheel == null) {
            throttleWheel = new XboxController(OIConstants.THROTTLE_WHEEL_ID);
        }
        return throttleWheel;
    }

    public XboxController getSteeringWheel() {
        if (steeringWheel == null) {
            steeringWheel = new XboxController(OIConstants.STEERING_WHEEL_ID);
        }
        return steeringWheel;
    }

    public XboxController getOperatorController() {
        if (operatorController == null) {
            operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER_ID);
        }

        return operatorController;
    }

    public XboxController getDriveController() {
        if(driveTestingController == null){
            driveTestingController = new XboxController(OIConstants.DRIVER_CONTROLLER);
        }

        return driveTestingController;
    }

    public PS4Controller getPS4Controller() {
        if(driverController == null){
            driverController = new PS4Controller(OIConstants.DRIVER_CONTROLLER);
        }

        return driverController;
    }

    public void setupControls() {
        XboxController controller = getOperatorController();

        Trigger coneMode = new Trigger(() -> getOperatorController().getRightTriggerAxis() > 0.5);
        coneMode.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateCone));

        Trigger cubeMode = new Trigger(() -> getOperatorController().getLeftTriggerAxis() > 0.5);
        cubeMode.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateCube));

        Trigger none = new Trigger(() -> getOperatorController().getRightTriggerAxis() < 0.5 && getOperatorController().getLeftTriggerAxis() < 0.5);
        none.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateNone));

        Trigger groundKinematics = new Trigger(getOperatorController()::getAButton);
        groundKinematics.whileTrue(new InstantCommand(ArmKinematics::handleGround));

        Trigger midKinematics = new Trigger(getOperatorController()::getXButton);
        midKinematics.whileTrue(new InstantCommand(ArmKinematics::handleGround));

        Trigger highKinematics = new Trigger(getOperatorController()::getYButton);
        highKinematics.whileTrue(new InstantCommand(ArmKinematics::handleGround));

        Trigger humanPlayerKinematics = new Trigger(() -> getOperatorController().getBButton() &&
                StateMachine.getInstance().getCurrentState() == StateMachine.State.CONE);
        humanPlayerKinematics.whileTrue(new InstantCommand(ArmKinematics::handleGround));

        Trigger intake = new Trigger(() -> TelescopeSubsystem.getInstance().withinThreshold() &&
                PivotSubsystem.getInstance().withinThreshold() && getOperatorController().getAButton());
        intake.whileTrue(new IntakeAnyLevelCommand());

        /*
        //incorporated into joystick throttle command
        Trigger lock0 = new Trigger(getDriveController()::getYButton);
        lock0.whileTrue(new SnapToAngle(0));

        Trigger lock90 = new Trigger(getDriveController()::getBButton);
        lock90.whileTrue(new SnapToAngle(3));

        Trigger lock180 = new Trigger(getDriveController()::getAButton);
        lock180.whileTrue(new SnapToAngle(2));

        Trigger lock270 = new Trigger(getDriveController()::getXButton);
        lock270.whileTrue(new SnapToAngle(1));
        */
    }

    public void setUpTuningControls() {
        setupControls();
    }

    private void triggerFunctionAfterTime(Runnable runnable, long time){
        new java.util.Timer().schedule(
                new java.util.TimerTask() {
                    @Override
                    public void run() {
                        runnable.run();
                    }
                },
                time
        );
    }
}
