package com.github.mittyrobotics.arm;

import com.github.mittyrobotics.arm.pivot.PivotConstants;
import com.github.mittyrobotics.arm.televator.TelevatorConstants;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;

import java.util.HashMap;

import static com.github.mittyrobotics.arm.StateMachine.TransitionState.*;

public class ArmMotionProfiles {
    public static final HashMap<StateMachine.TransitionState, TrapezoidalMotionProfile> PIVOT_MPS = new HashMap<>();
    public static final HashMap<StateMachine.TransitionState, TrapezoidalMotionProfile> TELEVATOR_MPS = new HashMap<>();

    public static void createMPs() {
        createPivotMPs();
        createTelevatorMPs();
    }

    private static void createPivotMPs() {
        PIVOT_MPS.put(STOWED_TO_LOW,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
        PIVOT_MPS.put(STOWED_TO_MID,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
        PIVOT_MPS.put(STOWED_TO_HIGH,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
        PIVOT_MPS.put(STOWED_TO_HP,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
        PIVOT_MPS.put(LOW_TO_STOWED,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
        PIVOT_MPS.put(MID_TO_STOWED,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
        PIVOT_MPS.put(HIGH_TO_STOWED,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
        PIVOT_MPS.put(HP_TO_STOWED,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
        PIVOT_MPS.put(MID_TO_HIGH,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
        PIVOT_MPS.put(HIGH_TO_MID,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
        PIVOT_MPS.put(SCORING_TO_STOWED,
                new TrapezoidalMotionProfile(1, 1, 6, 0, 0, 0, 0.1, 0.3));
    }

    private static void createTelevatorMPs() {
        TELEVATOR_MPS.put(STOWED_TO_LOW,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
        TELEVATOR_MPS.put(STOWED_TO_MID,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
        TELEVATOR_MPS.put(STOWED_TO_HIGH,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
        TELEVATOR_MPS.put(STOWED_TO_HP,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
        TELEVATOR_MPS.put(LOW_TO_STOWED,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
        TELEVATOR_MPS.put(MID_TO_STOWED,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
        TELEVATOR_MPS.put(HIGH_TO_STOWED,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
        TELEVATOR_MPS.put(HP_TO_STOWED,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
        TELEVATOR_MPS.put(MID_TO_HIGH,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
        TELEVATOR_MPS.put(HIGH_TO_MID,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
        TELEVATOR_MPS.put(SCORING_TO_STOWED,
                new TrapezoidalMotionProfile(1, 1, 5, 0, 0, 0, 1, 0.2));
    }
}
