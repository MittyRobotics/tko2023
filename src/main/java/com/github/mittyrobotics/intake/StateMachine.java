package com.github.mittyrobotics.intake;

public class StateMachine {
    private RobotState currentRobotState = RobotState.STOWED;

    private PieceState currentPieceState = PieceState.NONE;
    private PieceState lastPieceState;
    private IntakeState intakeState = IntakeState.OFF;

    private ProfileState profileState = ProfileState.DEFAULT;
    
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

    public void setStateScoring() {
        currentRobotState = RobotState.SCORING;
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

    public ProfileState getProfile() {
        return profileState;
    }

    public void setProfile(RobotState source, RobotState target) {
        if (source == RobotState.STOWED) setProfileFromStowed(target);
        else if (target == RobotState.STOWED) setProfileToStowed(source);
        else if (source == RobotState.MID && target == RobotState.HIGH) profileState = ProfileState.MID_TO_HIGH;
        else if (target == RobotState.MID && source == RobotState.HIGH) profileState = ProfileState.HIGH_TO_MID;
        else profileState = ProfileState.DEFAULT;
    }

    private void setProfileToStowed(RobotState source) {
        switch (source) {
            case GROUND:
                profileState = ProfileState.GROUND_TO_STOWED;
                break;
            case MID:
                profileState = ProfileState.MID_TO_STOWED;
                break;
            case HIGH:
                profileState = ProfileState.HIGH_TO_STOWED;
                break;
            case HP:
                profileState = ProfileState.HP_TO_STOWED;
                break;
            case SCORING:
                profileState = ProfileState.SCORING_TO_STOWED;
                break;
            default:
                profileState = ProfileState.DEFAULT;
        }
    }

    private void setProfileFromStowed(RobotState target) {
        switch (target) {
            case GROUND:
                profileState = ProfileState.STOWED_TO_GROUND;
                break;
            case MID:
                profileState = ProfileState.STOWED_TO_MID;
                break;
            case HIGH:
                profileState = ProfileState.STOWED_TO_HIGH;
                break;
            case HP:
                profileState = ProfileState.STOWED_TO_HP;
                break;
            default:
                profileState = ProfileState.DEFAULT;
        }
    }

    public enum RobotState {
        GROUND,
        MID,
        HIGH,
        HP,
        STOWED,
        SCORING
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

    public enum ProfileState {
        STOWED_TO_GROUND,
        STOWED_TO_MID,
        STOWED_TO_HIGH,
        STOWED_TO_HP,
        GROUND_TO_STOWED,
        MID_TO_STOWED,
        HIGH_TO_STOWED,
        HP_TO_STOWED,
        MID_TO_HIGH,
        HIGH_TO_MID,
        SCORING_TO_STOWED,
        DEFAULT
    }
}
