package com.github.mittyrobotics.arm;

import com.github.mittyrobotics.arm.pivot.PivotConstants;
import com.github.mittyrobotics.arm.televator.TelevatorConstants;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;

import java.util.HashMap;

import static com.github.mittyrobotics.arm.StateMachine.TransitionState.*;

public class MotionProfiles {
    public static final HashMap<StateMachine.TransitionState, TrapezoidalMotionProfile> PIVOT_MPS = new HashMap<>();
    public static final HashMap<StateMachine.TransitionState, TrapezoidalMotionProfile> TELEVATOR_MPS = new HashMap<>();

    public static void createMPs() {
        createPivotMPs();
        createTelevatorMPs();
    }

    private static void createPivotMPs() {
        PIVOT_MPS.put(STOWED_TO_LOW,
                new TrapezoidalMotionProfile(1200 / 360. / PivotConstants.RADIANS_PER_REV, 150 / 360. / PivotConstants.RADIANS_PER_REV, 860 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
        PIVOT_MPS.put(STOWED_TO_MID,
                new TrapezoidalMotionProfile(1200 / 360. / PivotConstants.RADIANS_PER_REV, 250 / 360. / PivotConstants.RADIANS_PER_REV, 860 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
        PIVOT_MPS.put(STOWED_TO_HIGH,
                new TrapezoidalMotionProfile(1200 / 360. / PivotConstants.RADIANS_PER_REV, 250 / 360. / PivotConstants.RADIANS_PER_REV, 860 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
        PIVOT_MPS.put(STOWED_TO_HP,
                new TrapezoidalMotionProfile(1000 / 360. / PivotConstants.RADIANS_PER_REV, 250 / 360. / PivotConstants.RADIANS_PER_REV, 720 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
        PIVOT_MPS.put(LOW_TO_STOWED,
                new TrapezoidalMotionProfile(800 / 360. / PivotConstants.RADIANS_PER_REV, 210 / 360. / PivotConstants.RADIANS_PER_REV, 720 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
        PIVOT_MPS.put(MID_TO_STOWED,
                new TrapezoidalMotionProfile(800 / 360. / PivotConstants.RADIANS_PER_REV, 210 / 360. / PivotConstants.RADIANS_PER_REV, 360 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
        PIVOT_MPS.put(HIGH_TO_STOWED,
                new TrapezoidalMotionProfile(800 / 360. / PivotConstants.RADIANS_PER_REV, 210 / 360. / PivotConstants.RADIANS_PER_REV, 360 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
        PIVOT_MPS.put(HP_TO_STOWED,
                new TrapezoidalMotionProfile(800 / 360. / PivotConstants.RADIANS_PER_REV, 210 / 360. / PivotConstants.RADIANS_PER_REV, 720 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
        PIVOT_MPS.put(MID_TO_HIGH,
                new TrapezoidalMotionProfile(400 / 360. / PivotConstants.RADIANS_PER_REV, 210 / 360. / PivotConstants.RADIANS_PER_REV, 720 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
        PIVOT_MPS.put(HIGH_TO_MID,
                new TrapezoidalMotionProfile(400 / 360. / PivotConstants.RADIANS_PER_REV, 210 / 360. / PivotConstants.RADIANS_PER_REV, 720 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
        PIVOT_MPS.put(SCORING_TO_STOWED,
                new TrapezoidalMotionProfile(400 / 360. / PivotConstants.RADIANS_PER_REV, 150 / 360. / PivotConstants.RADIANS_PER_REV, 360 / 360. / PivotConstants.RADIANS_PER_REV, 0, 0, 0 / 360. / PivotConstants.RADIANS_PER_REV, 30 / 360. / PivotConstants.RADIANS_PER_REV, 0.3));
    }

    private static void createTelevatorMPs() {
        TELEVATOR_MPS.put(STOWED_TO_LOW,
                new TrapezoidalMotionProfile(20 / 39.37 / TelevatorConstants.INCHES_PER_REV, 20 / 39.37 / TelevatorConstants.INCHES_PER_REV, 60 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 10 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
        TELEVATOR_MPS.put(STOWED_TO_MID,
                new TrapezoidalMotionProfile(20 / 39.37 / TelevatorConstants.INCHES_PER_REV, 20 / 39.37 / TelevatorConstants.INCHES_PER_REV, 240 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 10 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
        TELEVATOR_MPS.put(STOWED_TO_HIGH,
                new TrapezoidalMotionProfile(20 / 39.37 / TelevatorConstants.INCHES_PER_REV, 10 / 39.37 / TelevatorConstants.INCHES_PER_REV, 240 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 10 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
        TELEVATOR_MPS.put(STOWED_TO_HP,
                new TrapezoidalMotionProfile(20 / 39.37 / TelevatorConstants.INCHES_PER_REV, 10 / 39.37 / TelevatorConstants.INCHES_PER_REV, 120 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 10 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
        TELEVATOR_MPS.put(LOW_TO_STOWED,
                new TrapezoidalMotionProfile(40 / 39.37 / TelevatorConstants.INCHES_PER_REV, 30 / 39.37 / TelevatorConstants.INCHES_PER_REV, 120 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
        TELEVATOR_MPS.put(MID_TO_STOWED,
                new TrapezoidalMotionProfile(60 / 39.37 / TelevatorConstants.INCHES_PER_REV, 30 / 39.37 / TelevatorConstants.INCHES_PER_REV, 120 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
        TELEVATOR_MPS.put(HIGH_TO_STOWED,
                new TrapezoidalMotionProfile(60 / 39.37 / TelevatorConstants.INCHES_PER_REV, 30 / 39.37 / TelevatorConstants.INCHES_PER_REV, 120 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
        TELEVATOR_MPS.put(HP_TO_STOWED,
                new TrapezoidalMotionProfile(40 / 39.37 / TelevatorConstants.INCHES_PER_REV, 20 / 39.37 / TelevatorConstants.INCHES_PER_REV, 120 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
        TELEVATOR_MPS.put(MID_TO_HIGH,
                new TrapezoidalMotionProfile(70 / 39.37 / TelevatorConstants.INCHES_PER_REV, 30 / 39.37 / TelevatorConstants.INCHES_PER_REV, 120 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 10 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
        TELEVATOR_MPS.put(HIGH_TO_MID,
                new TrapezoidalMotionProfile(40 / 39.37 / TelevatorConstants.INCHES_PER_REV, 30 / 39.37 / TelevatorConstants.INCHES_PER_REV, 120 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
        TELEVATOR_MPS.put(SCORING_TO_STOWED,
                new TrapezoidalMotionProfile(80 / 39.37 / TelevatorConstants.INCHES_PER_REV, 20 / 39.37 / TelevatorConstants.INCHES_PER_REV, 160 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0, 0, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0 / 39.37 / TelevatorConstants.INCHES_PER_REV, 0.2));
    }
}
