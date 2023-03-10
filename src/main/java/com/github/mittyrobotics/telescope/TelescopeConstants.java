package com.github.mittyrobotics.telescope;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.util.TrapezoidalMotionProfile;

import java.util.HashMap;

public class TelescopeConstants {
    //TODO: Fill in all except meters per rev conversion
    public static final int TELESCOPE_SPARK_ID = 10;

    public static final double DEFAULT_P = 0.000;
    public static final double DEFAULT_I = 0;
    public static final double DEFAULT_D = 0;

    public static final double METERS_PER_MOTOR_REV = 2.75/39.37; //includes gear ratio and two stage
    public static final double MAX_VELOCITY = 0;
    public static final double MIN_VELOCITY = 0;
    public static final double MAX_ACCEL = 0;

    public static final double EXTENSION_THRESHOLD = 0;

    public static final int HALIFAX_MAX_CHANNEL = 6;
    public static final int HALIFAX_MIN_CHANNEL = 8;

    public static final double MAX_EXTENSION_METERS = 1.0111647328053874;

    public static final HashMap<StateMachine.ProfileState, TrapezoidalMotionProfile> TELESCOPE_MPS = new HashMap<>();
    //2.75 inches every MOTOR rotation
    //4 motor rotations for 5.5 inches (times 2, since 2 stage)
}
