package com.github.mittyrobotics.arm;

import com.github.mittyrobotics.intake.IntakeSubsystem;

import static com.github.mittyrobotics.arm.ArmKinematics.ArmPosition;

public class StateMachine {
    private static ArmState currentArmState;
    private static PieceState pieceState;
    private static TransitionState transitionState;
    private static IntakeState intakeState;

    public static void init() {
        setArmState(ArmState.STOWED);
        setPieceState(PieceState.NONE);
        setTransitionState(ArmState.STOWED, ArmState.HIGH);
    }

    public static void handleStowed() {
        setTransitionState(getArmState(), ArmState.STOWED);
        setArmState(ArmState.STOWED);
        setIntakeState(IntakeSubsystem.getInstance().intakeFull() ?
                IntakeState.STOWING : IntakeState.EMPTY);
    }

    public static void handleLow() {
        setTransitionState(getArmState(), ArmState.LOW);
        setArmState(ArmState.LOW);
        setIntakeState(IntakeState.INTAKING);
    }

    public static void handleMid() {
        setTransitionState(getArmState(), ArmState.MID);
        setArmState(ArmState.MID);
    }

    public static void handleHigh() {
        setTransitionState(getArmState(), ArmState.HIGH);
        setArmState(ArmState.HIGH);
    }

    public static void handleHP() {
        setTransitionState(getArmState(), ArmState.HP);
        setArmState(ArmState.HP);
        setIntakeState(IntakeState.INTAKING);
    }

    public static void handleScore() {
        setTransitionState(getArmState(), ArmState.SCORING);
        setArmState(ArmState.SCORING);
    }

    public static void setArmState(ArmState state) {
        currentArmState = state;
    }

    public static ArmState getArmState() {
        return currentArmState;
    }

    public static void setPieceState(PieceState state) {
        pieceState = state;
    }

    public static PieceState getPieceState() {
        return pieceState;
    }

    public static void setTransitionState(ArmState from, ArmState to) {
        if (from == ArmState.STOWED) {
            switch (to) {
                case LOW:
                    transitionState = TransitionState.STOWED_TO_LOW;
                    break;
                case MID:
                    transitionState = TransitionState.STOWED_TO_MID;
                    break;
                case HIGH:
                    transitionState = TransitionState.STOWED_TO_HIGH;
                    break;
                case HP:
                    transitionState = TransitionState.STOWED_TO_HP;
                    break;
                default:
                    transitionState = TransitionState.STOWED_TO_HIGH;
            }
        } else if (to == ArmState.STOWED) {
            switch (from) {
                case LOW:
                    transitionState = TransitionState.LOW_TO_STOWED;
                    break;
                case MID:
                    transitionState = TransitionState.MID_TO_STOWED;
                    break;
                case HIGH:
                    transitionState = TransitionState.HIGH_TO_STOWED;
                    break;
                case HP:
                    transitionState = TransitionState.HP_TO_STOWED;
                    break;
                case SCORING:
                    transitionState = TransitionState.SCORING_TO_STOWED;
                    break;
                default:
                    transitionState = TransitionState.SCORING_TO_STOWED;
            }
        } else if (from == ArmState.MID) transitionState = TransitionState.MID_TO_HIGH;
        else if (from == ArmState.HIGH) transitionState = TransitionState.HIGH_TO_MID;
    }

    public static TransitionState getTransitionState() {
        return transitionState;
    }

    public static void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    public static IntakeState getIntakeState() {
        return intakeState;
    }

    public static boolean withinThreshold() {
        return withinThreshold(2, 0.5);
    }

    public static boolean withinThreshold(double angleThreshold, double extensionThreshold) {
        ArmPosition diff = ArmPosition.getDifference(
                ArmSetpoints.positions.get(getArmState()).get(getPieceState()),
                ArmKinematics.getCurrentArmPosition()
        );
        return diff.getAngle().getDegrees() < angleThreshold && diff.getRadius() < extensionThreshold;
    }


    public enum ArmState {
        STOWED,
        HIGH,
        MID,
        LOW,
        HP,
        SCORING,
        RETRACTED
    }

    public enum PieceState {
        CONE,
        CUBE,
        NONE
    }

    public enum TransitionState {
        STOWED_TO_LOW,
        STOWED_TO_MID,
        STOWED_TO_HIGH,
        STOWED_TO_HP,
        LOW_TO_STOWED,
        MID_TO_STOWED,
        HIGH_TO_STOWED,
        HP_TO_STOWED,
        MID_TO_HIGH,
        HIGH_TO_MID,
        SCORING_TO_STOWED,
    }

    public enum IntakeState {
        EMPTY,
        INTAKING,
        STOWING,
        OUTTAKING
    }
}
