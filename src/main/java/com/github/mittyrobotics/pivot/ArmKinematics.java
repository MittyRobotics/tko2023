package com.github.mittyrobotics.pivot;

import com.github.mittyrobotics.StateMachine;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;

public class ArmKinematics {
    private static Angle pitch;
    private static double radius;

    public static void setArmKinematics(double distance, double height) {
        radius = Math.sqrt(distance * distance + height * height);
        pitch = new Angle(Math.atan2(height, distance));
    }

    public static Angle getPivotDesired() {
        return pitch;
    }

    public static double getTelescopeDesired() {
        return radius;
    }

    public static void handleGround() {
        if (StateMachine.getInstance().getCurrentState() != StateMachine.State.NONE)
            setArmKinematics(PivotConstants.DISTANCE_TO_LOW_SCORE,
                    PivotConstants.LOW_HEIGHT - PivotConstants.PIVOT_HEIGHT);
    }

    public static void handleMid() {
        if (StateMachine.getInstance().getCurrentState() != StateMachine.State.NONE)
            setArmKinematics(PivotConstants.DISTANCE_TO_MID_SCORE,
                    PivotConstants.MID_HEIGHT - PivotConstants.PIVOT_HEIGHT);
    }

    public static void handleHigh() {
        if (StateMachine.getInstance().getCurrentState() != StateMachine.State.NONE)
            setArmKinematics(PivotConstants.DISTANCE_TO_HIGH_SCORE,
                    PivotConstants.HIGH_HEIGHT - PivotConstants.PIVOT_HEIGHT);
    }

    public static void handleHumanPlayer() {
        if (StateMachine.getInstance().getCurrentState() != StateMachine.State.NONE)
            setArmKinematics(PivotConstants.DISTANCE_TO_HP_SCORE,
                    PivotConstants.HP_HEIGHT - PivotConstants.PIVOT_HEIGHT);
    }
}