package com.github.mittyrobotics.intake;

public class StateMachine {
    private RobotState currentRobotState = RobotState.STOWED;

    private PieceState currentPieceState = PieceState.NONE;
    private PieceState lastPieceState;
    private IntakeState intakeState = IntakeState.STOW;
    
    public static StateMachine instance;

    public static StateMachine getInstance() {
        if(instance == null) {
            instance = new StateMachine();
        }
        return instance;
    }

    public RobotState getCurrentRobotState() {
        return currentRobotState;
    }

    public void setStateStowed() {
        currentRobotState = RobotState.STOWED;
    }

    public void setStateGround() {
        currentRobotState = RobotState.GROUND;
    }

    public void setStateMid() {
        currentRobotState = RobotState.MID;
    }

    public void setStateHigh() {
        currentRobotState = RobotState.HIGH;
    }

    public void setStateHP() {
        currentRobotState = RobotState.HP;
    }

    public PieceState getCurrentPieceState() {
        return currentPieceState;
    }

    public PieceState getLastPieceState() {
        return lastPieceState;
    }

    public void setStateCube() {
        currentPieceState = PieceState.CUBE;
        lastPieceState = PieceState.CUBE;
        System.out.println("Set state cube");
    }

    public void setStateCone() {
        currentPieceState = PieceState.CONE;
        lastPieceState = PieceState.CONE;
    }

    public void setStateNone() {
        currentPieceState = PieceState.NONE;
    }

    public void setIntaking() {
        intakeState = IntakeState.INTAKE;
    }

    public void setIntakeStowing() {
        intakeState = IntakeState.STOW;
    }

    public void setIntakeOff() {
        intakeState = IntakeState.OFF;
    }

    public void setOuttaking() {
        intakeState = IntakeState.OUTTAKE;
    }

    public IntakeState getIntakingState() {
        return intakeState;
    }

    public enum RobotState {
        GROUND,
        MID,
        HIGH,
        HP,
        STOWED
    }

    public enum PieceState {
        CONE,
        CUBE,
        NONE
    }
    
    public enum IntakeState {
        INTAKE,
        STOW,
        OUTTAKE,
        OFF
    }

}
