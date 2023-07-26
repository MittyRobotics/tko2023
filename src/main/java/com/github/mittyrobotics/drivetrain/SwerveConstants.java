package com.github.mittyrobotics.drivetrain;

public class SwerveConstants {
    public static final int[] DRIVE_MOTOR_IDS = {};
    public static final int[] ANGLE_MOTOR_IDS = {};

    public static final double[] DRIVE_PID = {};
    public static final double[] ANGLE_PID = {};

    public static final boolean[] DRIVE_INVERTED = {};
    public static final boolean[] ANGLE_INVERTED = {};

    private static final double RADIUS_OF_WHEEL = 2. * 2.54 / 100.;

    private static final double DRIVE_WHEEL_TO_FALCON_GEAR_RATIO = 1. / 6.54;
    private static final double MODULE_TO_FALCON_GEAR_RATIO = 7./108.;

    private static final double FALCON_TICKS = 2048.;

    private static final double TICKS_PER_METER = FALCON_TICKS / DRIVE_WHEEL_TO_FALCON_GEAR_RATIO / (2 * Math.PI * RADIUS_OF_WHEEL);

    public static final double TICKS_PER_INCH = TICKS_PER_METER * 39.37;

    public static final double TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO = FALCON_TICKS / MODULE_TO_FALCON_GEAR_RATIO / (2 * Math.PI);

}
