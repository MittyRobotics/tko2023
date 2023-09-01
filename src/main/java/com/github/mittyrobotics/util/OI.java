package com.github.mittyrobotics.util;

import com.github.mittyrobotics.arm.StateMachine;
import com.github.mittyrobotics.autonomous.actions.AutoScoreCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.github.mittyrobotics.util.OIConstants.*;

public class OI {
    private static OI instance;

    public static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }

    private XboxController driverController, operatorController;

    public void initHardware() {
        driverController = new XboxController(DRIVER_CONTROLLER_ID);
        operatorController = new XboxController(OPERATOR_CONTROLLER_ID);
    }

    public XboxController getDriverController() {
        return driverController;
    }

    public XboxController getOperatorController() {
        return operatorController;
    }

    private boolean pieceControls(boolean left, boolean right) {
        return
                left == operatorController.getLeftTriggerAxis() > 0.5 &&
                right == operatorController.getRightTriggerAxis() > 0.5;
    }

    private boolean operatorControls(boolean a, boolean b, boolean x, boolean y) {
        return
                a == operatorController.getAButton() &&
                b == operatorController.getBButton() &&
                x == operatorController.getXButton() &&
                y == operatorController.getYButton();
    }

    public void setupControls() {
        Trigger coneMode = new Trigger(() -> pieceControls(true, false));
        coneMode.whileTrue(new InstantCommand(() -> StateMachine.setPieceState(StateMachine.PieceState.CONE)));

        Trigger cubeMode = new Trigger(() -> pieceControls(false, true));
        cubeMode.whileTrue(new InstantCommand(() -> StateMachine.setPieceState(StateMachine.PieceState.CUBE)));

        Trigger noneMode = new Trigger(() -> pieceControls(false, false) &&
                StateMachine.getDesiredArmState() != StateMachine.ArmState.HIGH &&
                StateMachine.getDesiredArmState() != StateMachine.ArmState.MID);
        noneMode.whileTrue(new InstantCommand(() -> StateMachine.setPieceState(StateMachine.PieceState.NONE)));


        Trigger armStowed = new Trigger(() -> operatorControls(false, false, false, false));
        armStowed.whileTrue(new InstantCommand(StateMachine::handleStowed));

        Trigger armLow = new Trigger(() -> operatorControls(true, false, false, false));
        armLow.whileTrue(new InstantCommand(StateMachine::handleLow));

        Trigger armHP = new Trigger(() -> operatorControls(false, true, false, false));
        armHP.whileTrue(new InstantCommand(StateMachine::handleHP));

        Trigger armMid = new Trigger(() -> operatorControls(false, false, true, false));
        armMid.whileTrue(new InstantCommand(StateMachine::handleMid));

        Trigger armHigh = new Trigger(() -> operatorControls(false, false, false, true));
        armHigh.whileTrue(new InstantCommand(StateMachine::handleHigh));


        Trigger score = new Trigger(() -> pieceControls(false, false) &&
                (StateMachine.getDesiredArmState() == StateMachine.ArmState.HIGH ||
                 StateMachine.getDesiredArmState() == StateMachine.ArmState.MID));
        score.whileTrue(new AutoScoreCommand());
    }
}
