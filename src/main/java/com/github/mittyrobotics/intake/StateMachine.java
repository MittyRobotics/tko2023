package com.github.mittyrobotics.intake;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;

public class StateMachine {
    private RobotState currentRobotState = RobotState.STOWED;

    private PieceState currentPieceState = PieceState.NONE;
    private PieceState lastPieceState;

    boolean isIntaking = false;
    long time;

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

    public boolean readyToShoot() {
        return PivotSubsystem.getInstance().withinThreshold() && TelescopeSubsystem.getInstance().withinThreshold();
    }

    public Pose getNearestConePose() {
        return SwerveSubsystem.getInstance().getPose();
    }

    public Pose getNearestCubePose() {
        return SwerveSubsystem.getInstance().getPose();
    }

    public Pose getNearestGamePiecePose() {
        if (currentPieceState == PieceState.NONE) {
            //FIX
            return SwerveSubsystem.getInstance().getPose();
        }
        return currentPieceState == PieceState.CUBE ? getNearestCubePose() : getNearestConePose();
    }

    public void update() {

    }

    public void setIntaking(boolean isIntaking) {
        this.isIntaking = isIntaking;
    }

    public boolean getIntaking() {
        return isIntaking;
    }

    public boolean shouldBeIntaking() {
        return System.currentTimeMillis() - time < 1000;
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

}
