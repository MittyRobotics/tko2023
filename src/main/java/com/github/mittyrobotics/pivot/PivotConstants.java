package com.github.mittyrobotics.pivot;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;

import java.util.HashMap;

public class PivotConstants {
    public static final int[] PIVOT_ID = {8, 9};

    public static final double PIVOT_BASE_P = 7;
    public static final double PIVOT_BASE_I = 0.000;
    public static final double PIVOT_BASE_D = 0.1;

    public static final double MAX_ACCEL = 0;
    public static final double MAX_VEL = 0;

    public static final double PIVOT_TO_NEO_GEAR_RATIO = 11. / 882.;

    public static final double PIVOT_HEIGHT = 0;

    public static final double DISTANCE_TO_LOW_SCORE = 0;
    public static final double DISTANCE_TO_MID_SCORE = 0;
    public static final double DISTANCE_TO_HIGH_SCORE = 0;
    public static final double DISTANCE_TO_HP_SCORE = 0;

    public static final double LOW_HEIGHT = 0;
    public static final double MID_HEIGHT = 0;
    public static final double HIGH_HEIGHT = 0;
    public static final double HP_HEIGHT = 0;

    public static final double PIVOT_THRESHOLD = 0;


    public static final int HALIFAX_TOP_CHANNEL = 7;
    public static final int HALIFAX_BOTTOM_CHANNEL = 9;

    public static final double HALIFAX_TOP_DEGREES = 18.26;
    public static final double HALIFAX_BOTTOM_DEGREES = -18.26;

    public static final double SOFT_LIMIT_TOP_RADIANS = Math.PI/4;
    public static final double SOFT_LIMIT_BOTTOM_RADIANS = -Math.PI/4;

    public static final HashMap<StateMachine.ProfileState, TrapezoidalMotionProfile> PIVOT_MPS = new HashMap<>();
}
