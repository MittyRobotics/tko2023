package com.github.mittyrobotics.pivot;

import com.github.mittyrobotics.StateMachine;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;

public class ArmKinematics {
    private static Angle pitch = new Angle(0);

    static double incrementSpeed = 0.0005;

    private static double tuningHeight = 5/39.37;
    private static double radius = 0;
    private static double tuningDistance = 0;

    public static void setArmKinematics(double distance, double height) {
        radius = Math.sqrt(distance * distance + height * height);
        pitch = new Angle(Math.atan2(height, distance));
    }

    public static void setArmKinematics(Angle theta, double r) {
        radius = r;
        pitch = theta;
    }

    public static Angle getPivotDesiredCartesian() {
        return new Angle(Math.PI/2 - pitch.getRadians());
    }

    public static Angle getPivotDesiredPolar() {
        return pitch;
    }

    public static double getTelescopeDesired() {
        return radius;
    }

    public static void handleHigh() {
        if (StateMachine.getInstance().getCurrentState() != StateMachine.State.NONE)
            setArmKinematics(new Angle(1.1828889648626106), 0.9712510524378655);
    }

    public static void handleMid() {
        if (StateMachine.getInstance().getCurrentState() != StateMachine.State.NONE)
            setArmKinematics(new Angle(1.1866203864449478), 0.545497612205895);
    }

    public static void handleGround() {
        if (StateMachine.getInstance().getCurrentState() != StateMachine.State.NONE){

        }

    }

    public static void handleHumanPlayer() {
        if (StateMachine.getInstance().getCurrentState() != StateMachine.State.NONE)
            setArmKinematics(new Angle(1.076542665084606), 0.1812776848238557);
    }

    public static void incrementDistance(boolean extend) {
        System.out.println("TUNING DISTANCE: " + tuningHeight);
        if (extend) {
            setArmKinematics(tuningDistance+=incrementSpeed, tuningHeight);
        } else {
            setArmKinematics(tuningDistance-=incrementSpeed, tuningHeight);
        }


    }

    public static void incrementHeight(boolean up) {
        System.out.println("TUNING HEIGHT: " + tuningHeight);
        if (up) {
            setArmKinematics(tuningDistance, tuningHeight+=incrementSpeed);
        } else {
            setArmKinematics(tuningDistance, tuningHeight-=incrementSpeed);
        }
    }

    public static void incrementLinear(double joystickY, double joystickX) {
        setArmKinematics(0.75 * Math.pow(joystickX,4), Math.pow(joystickY, 4)*0.75);
    }
}
