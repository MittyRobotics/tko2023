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

import com.github.mittyrobotics.autonomous.pathfollowing.SwerveDriverPurePursuitCommand;
import com.github.mittyrobotics.drivetrain.commands.JoystickThrottleCommand;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.intake.IntakeSubsystem;
import com.github.mittyrobotics.led.commands.PurpleCommand;
import com.github.mittyrobotics.led.commands.YellowCommand;
import com.github.mittyrobotics.pivot.ArmKinematics;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * OI Class to manage all controllers and input
 */
public class OI {
    private static OI instance;

    private PS4Controller driverPS4Controller;

    private XboxController throttleWheel;
    private XboxController steeringWheel;
    private XboxController operatorController;
    private XboxController driverXboxController;

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public XboxController getOperatorController() {
        if (operatorController == null) {
            operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER_ID);
        }

        return operatorController;
    }

    public XboxController getDriveController() {
        if(driverXboxController == null){
            driverXboxController = new XboxController(OIConstants.DRIVER_CONTROLLER);
        }

        return driverXboxController;
    }

    public PS4Controller getPS4Controller() {
        if(driverPS4Controller == null){
            driverPS4Controller = new PS4Controller(OIConstants.DRIVER_CONTROLLER);
        }

        return driverPS4Controller;
    }

    public void zeroAll() {
        ArmKinematics.setArmKinematics(new Angle(0), 0);
        StateMachine.getInstance().setStateStowed();
    }

    public void handleGround() {
        if (StateMachine.getInstance().getCurrentPieceState() != StateMachine.PieceState.NONE) return;
        ArmKinematics.setArmKinematics(new Angle(2.1847833197051916), 0.20456099255847424);
        StateMachine.getInstance().setStateGround();
    }

    public void handleMid() {
        if (StateMachine.getInstance().getCurrentPieceState() != StateMachine.PieceState.NONE) return;
        ArmKinematics.setArmKinematics(new Angle(1.173560373540987), 0.545497612205895 - 2 / 39.37);
        StateMachine.getInstance().setStateMid();
    }

    public void handleHigh() {
        if (StateMachine.getInstance().getCurrentPieceState() != StateMachine.PieceState.NONE) return;
        ArmKinematics.setArmKinematics(new Angle(1.0933341743024654 - 0.04 - 0.035), 0.9712510524378655 - 2 / 39.27);
        StateMachine.getInstance().setStateHigh();
    }

    public void handleHumanPlayer() {
        if (StateMachine.getInstance().getCurrentPieceState() != StateMachine.PieceState.NONE) return;
        ArmKinematics.setArmKinematics(new Angle(1.113857105102662 - 2 * Math.PI/180), 0.4490367646201602);
        StateMachine.getInstance().setStateHP();
    }

    public boolean driverControls(boolean leftBumper, boolean rightBumper, boolean leftTrigger, boolean rightTrigger) {
        return (leftBumper && getOperatorController().getLeftBumper()) &&
                (rightBumper && getOperatorController().getRightBumper()) &&
                (leftTrigger && getOperatorController().getLeftTriggerAxis() > 0.5) &&
                (rightTrigger && getOperatorController().getRightTriggerAxis() > 0.5);
    }

    public void setupControls() {
        Trigger coneMode = new Trigger(() -> getOperatorController().getRightTriggerAxis() > 0.5);
        coneMode.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateCone));

        Trigger cubeMode = new Trigger(() -> getOperatorController().getLeftTriggerAxis() > 0.5);
        cubeMode.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateCube));

        Trigger none = new Trigger(() -> getOperatorController().getRightTriggerAxis() < 0.5 && getOperatorController().getLeftTriggerAxis() < 0.5);
        none.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateNone));

        Trigger zeroAll = new Trigger(() -> StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.NONE);
        zeroAll.whileTrue(new InstantCommand(this::zeroAll));

        Trigger groundKinematics = new Trigger(getOperatorController()::getAButton);
        groundKinematics.whileTrue(new InstantCommand(this::handleGround));

        Trigger midKinematics = new Trigger(getOperatorController()::getXButton);
        midKinematics.whileTrue(new InstantCommand(this::handleMid));

        Trigger highKinematics = new Trigger(getOperatorController()::getYButton);
        highKinematics.whileTrue(new InstantCommand(this::handleHigh));

        Trigger humanPlayerKinematics = new Trigger(getOperatorController()::getBButton);
        humanPlayerKinematics.whileTrue(new InstantCommand(this::handleHumanPlayer));

        Trigger autoStow = new Trigger(() -> (
                StateMachine.getInstance().getCurrentRobotState() == StateMachine.RobotState.GROUND ||
                StateMachine.getInstance().getCurrentRobotState() == StateMachine.RobotState.HP) &&
                IntakeSubsystem.getInstance().isIntakeFull()
        );
        autoStow.whileTrue(new InstantCommand(this::zeroAll));

        Trigger autoIntakeGround = new Trigger(() -> driverControls(true, false, false, false));
//        autoIntakeGround.whileTrue(new SwerveDriverPurePursuitCommand());

        Trigger autoIntakeHP = new Trigger(() -> driverControls(false, true, false, false));
//        autoIntakeHP.whileTrue(new SwerveDriverPurePursuitCommand());

        Trigger autoLeft = new Trigger(() -> driverControls(false, false, true, false));
//        autoLeft.whileTrue(new SwerveDriverPurePursuitCommand());

        Trigger autoCenter = new Trigger(() -> driverControls(false, false, true, true));
//        autoCenter.whileTrue(new SwerveDriverPurePursuitCommand());

        Trigger autoRight = new Trigger(() -> driverControls(false, false, false, true));
//        autoRight.whileTrue(new SwerveDriverPurePursuitCommand());

        Trigger drive = new Trigger(() -> driverControls(false, false, false, false));
        drive.whileTrue(new JoystickThrottleCommand());
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
