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

import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.pivot.ArmKinematics;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        StateMachine.getInstance().setProfile(StateMachine.getInstance().getCurrentRobotState(), StateMachine.RobotState.STOWED);
        StateMachine.getInstance().setStateStowed();
    }

    public void handleGround() {
        if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CUBE)
            ArmKinematics.setArmKinematics(new Angle(2.1847833197051916 - 0.08), 0.20);
        if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CONE)
            ArmKinematics.setArmKinematics(new Angle(2.1847833197051916 - 0.), 0.17);
        StateMachine.getInstance().setProfile(StateMachine.getInstance().getCurrentRobotState(), StateMachine.RobotState.GROUND);
        StateMachine.getInstance().setStateGround();
        if(StateMachine.getInstance().getIntakingState() != StateMachine.IntakeState.STOW)
            StateMachine.getInstance().setIntaking();
    }

    public void handleMid() {
        if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CONE)
            ArmKinematics.setArmKinematics(new Angle(1.16 - 0.1), 0.527);
        if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CUBE)
            ArmKinematics.setArmKinematics(new Angle(1.304), 0);
        StateMachine.getInstance().setProfile(StateMachine.getInstance().getCurrentRobotState(), StateMachine.RobotState.MID);
        StateMachine.getInstance().setStateMid();
    }

    public void handleHigh() {
        if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CONE)
            ArmKinematics.setArmKinematics(new Angle(1.12 - 0.15), 0.9513 + 0.06);
        if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CUBE)
            ArmKinematics.setArmKinematics(new Angle(1.263 - 0.2 - 0.1), 0.494);
        StateMachine.getInstance().setProfile(StateMachine.getInstance().getCurrentRobotState(), StateMachine.RobotState.HIGH);
        StateMachine.getInstance().setStateHigh();
    }

    public void handleHumanPlayer() {
        ArmKinematics.setArmKinematics(new Angle(1.071 + 0.03), 0.479);
        StateMachine.getInstance().setProfile(StateMachine.getInstance().getCurrentRobotState(), StateMachine.RobotState.HP);
        StateMachine.getInstance().setStateHP();
        StateMachine.getInstance().setIntaking();
    }
    
    public void handleScore() {
        StateMachine.getInstance().setStateScoring();

        if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CUBE) {
            StateMachine.getInstance().setOuttaking();

            Util.triggerFunctionAfterTime(() -> {
                zeroAll();
                Util.triggerFunctionAfterTime(() -> {
                    StateMachine.getInstance().setIntakeOff();
                    StateMachine.getInstance().setStateNone();
//                    Odometry.getInstance().setScoringCam(false);
                }, 200);
            }, 300);
        } else {
            double curRad = ArmKinematics.getTelescopeDesired();
            double curAngle = ArmKinematics.getPivotDesired().getRadians();
            ArmKinematics.setArmKinematics(new Angle(curAngle + 25 * Math.PI/180), curRad);

            Util.triggerFunctionAfterTime(() -> {
                StateMachine.getInstance().setOuttaking();
                Util.triggerFunctionAfterTime(() -> {
                    ArmKinematics.setArmKinematics(new Angle(ArmKinematics.getPivotDesired().getRadians()), 0);
                    StateMachine.getInstance().setProfile(StateMachine.RobotState.SCORING, StateMachine.RobotState.STOWED);
                    Util.triggerFunctionAfterTime(() -> {
                        zeroAll();
                        StateMachine.getInstance().setProfile(StateMachine.RobotState.SCORING, StateMachine.RobotState.STOWED);
                        Util.triggerFunctionAfterTime(() -> {
                            StateMachine.getInstance().setIntakeOff();
                            StateMachine.getInstance().setStateNone();
                        }, 200);
//                        Odometry.getInstance().setScoringCam(false);
                    }, 1000);
                }, 10);
            }, 300);
        }
    }

    public boolean driverControls(boolean xButton, boolean bButton, boolean leftTrigger, boolean rightTrigger) {
        return (xButton == getDriveController().getXButton()) &&
                (bButton == getDriveController().getBButton()) &&
                (leftTrigger == getDriveController().getLeftTriggerAxis() > 0.5) &&
                (rightTrigger == getDriveController().getRightTriggerAxis() > 0.5);
    }

    public void setupControls() {
        Trigger coneMode = new Trigger(() -> getOperatorController().getRightTriggerAxis() > 0.5);
        coneMode.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateCone));

        Trigger cubeMode = new Trigger(() -> getOperatorController().getLeftTriggerAxis() > 0.5);
        cubeMode.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateCube));

        Trigger none = new Trigger(() ->
                (getOperatorController().getRightTriggerAxis() < 0.5 &&
                getOperatorController().getLeftTriggerAxis() < 0.5) &&
                StateMachine.getInstance().getCurrentRobotState() != StateMachine.RobotState.HIGH &&
                StateMachine.getInstance().getCurrentRobotState() != StateMachine.RobotState.MID &&
                StateMachine.getInstance().getCurrentRobotState() != StateMachine.RobotState.SCORING);
        none.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateNone));

        Trigger zeroAll = new Trigger(() -> StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.NONE);
        zeroAll.whileTrue(new InstantCommand(this::zeroAll));

        Trigger groundKinematics = new Trigger(getOperatorController()::getAButton);
        groundKinematics.whileTrue(new InstantCommand(this::handleGround));

        Trigger midKinematics = new Trigger(getOperatorController()::getXButton);
        midKinematics.whileTrue(new InstantCommand(this::handleMid));

        Trigger highKinematics = new Trigger(getOperatorController()::getYButton);
        highKinematics.whileTrue(new InstantCommand(this::handleHigh));

        Trigger readyToScore = new Trigger(() ->
                (StateMachine.getInstance().getCurrentRobotState() == StateMachine.RobotState.MID ||
                        StateMachine.getInstance().getCurrentRobotState() == StateMachine.RobotState.HIGH) &&
                        (getOperatorController().getRightTriggerAxis() < 0.5 &&
                                getOperatorController().getLeftTriggerAxis() < 0.5)
        );
        readyToScore.onTrue(new InstantCommand(this::handleScore));

        Trigger humanPlayerKinematics = new Trigger(getOperatorController()::getBButton);
        humanPlayerKinematics.whileTrue(new InstantCommand(this::handleHumanPlayer));
//
//        Trigger autoIntakeGround = new Trigger(() -> driverControls(true, false, false, false));
//        autoIntakeGround.whileTrue(new AutoPickupCommand(0, 0, 0, 0, 0, 0,  false));
//        autoIntakeGround.onFalse(new InstantCommand(() -> LedSubsystem.getInstance().disableDriveAltColor()));
//
//        Trigger autoIntakeHP = new Trigger(() -> driverControls(false, true, false, false)
//                && StateMachine.getInstance().getCurrentPieceState() != StateMachine.PieceState.NONE);
//        autoIntakeHP.whileTrue(new SwerveAutoPickupCommand(StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CONE, 0, false));
//        autoIntakeGround.onFalse(new InstantCommand(() -> LedSubsystem.getInstance().disableDriveAltColor()));
////
        Trigger autoRight = new Trigger(() -> driverControls(false, false, false, true));
//        autoRight.whileTrue(new TeleopScoreCommand(2));
//        autoRight.onFalse(new InstantCommand(() -> LedSubsystem.getInstance().disableDriveAltColor()));
//
        Trigger autoCenter = new Trigger(() -> driverControls(false, false, true, true));
//        autoCenter.whileTrue(new TeleopScoreCommand(1));
//        autoCenter.onFalse(new InstantCommand(() -> LedSubsystem.getInstance().disableDriveAltColor()));
//
        Trigger autoLeft = new Trigger(() -> driverControls(false, false, true, false));
//        autoLeft.whileTrue(new TeleopScoreCommand(0));
//        autoLeft.onFalse(new InstantCommand(() -> LedSubsystem.getInstance().disableDriveAltColor()));
//
//        Trigger zeroModules = new Trigger(() -> getOperatorController().getStartButton());
//        zeroModules.onTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().setAnglesZero()));

//        Trigger drive = new Trigger(() -> driverControls(false, false, false, false));
//        drive.whileTrue(new JoystickThrottleCommand());
    }

    public void setupTestLEDControls() {
        Trigger coneMode = new Trigger(() -> getOperatorController().getRightTriggerAxis() > 0.5);
        coneMode.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateCone));

        Trigger cubeMode = new Trigger(() -> getOperatorController().getLeftTriggerAxis() > 0.5);
        cubeMode.whileTrue(new InstantCommand(StateMachine.getInstance()::setStateCube));
    }
}
