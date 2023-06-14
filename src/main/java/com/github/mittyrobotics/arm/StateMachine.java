package com.github.mittyrobotics.arm;

import com.github.mittyrobotics.util.Util;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;

import static com.github.mittyrobotics.arm.ArmKinematics.ArmPosition;

public class StateMachine {
    private static Queue<ArmState> desiredArmStates = new LinkedList<>();
    private static ArmState lastArmState;
    private static long delay = 0;

    private static void initStateMachine() {
        desiredArmStates.add(ArmState.STOWED);
        lastArmState = ArmState.STOWED;
    }

    public static int addStateToQueue(ArmState state) {
        desiredArmStates.add(state);
        return desiredArmStates.size();
    }

    public static ArmState getDesiredArmState() {
        lastArmState = desiredArmStates.peek() == null ? lastArmState : desiredArmStates.peek();
        return lastArmState;
    }

    public static void setDelay(long delay) {
        StateMachine.delay = delay;
    }

    public static boolean withinThreshold(double angleThreshold, double extensionThreshold) {
        ArmPosition diff = ArmPosition.getDifference(
                ArmSetpoints.positions.get(getDesiredArmState()),
                ArmKinematics.getCurrentArmPosition()
        );
        return diff.getAngle().getDegrees() < angleThreshold && diff.getRadius() < extensionThreshold;
    }

    public static void update(double angleThreshold, double extensionThreshold) {
        if (withinThreshold(angleThreshold, extensionThreshold))
            Util.triggerFunctionAfterTime(
                    () -> {
                        desiredArmStates.poll();
                    },
                    delay
            );
    }
    public enum ArmState {
        STOWED,
        HIGH,
        MID,
        LOW

    }
    public enum IntakeState {
        EMPTY,
        INTAKING,
        STOWING,
        OUTTAKING
    }
}
