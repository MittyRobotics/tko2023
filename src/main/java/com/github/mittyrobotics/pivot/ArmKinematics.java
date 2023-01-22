package com.github.mittyrobotics.pivot;

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
        setArmKinematics(PivotConstants.DISTANCE_TO_LOW_SCORE, PivotConstants.PIVOT_HEIGHT);
    }

    public static void handleMid() {
        setArmKinematics(PivotConstants.DISTANCE_TO_LOW_SCORE, PivotConstants.PIVOT_HEIGHT);
    }

    public static void handleHigh() {
        setArmKinematics(PivotConstants.DISTANCE_TO_LOW_SCORE, PivotConstants.PIVOT_HEIGHT);
    }

    public static void handleHumanPlayer() {
        setArmKinematics(PivotConstants.DISTANCE_TO_LOW_SCORE, PivotConstants.PIVOT_HEIGHT);
    }
}
